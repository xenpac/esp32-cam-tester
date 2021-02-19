#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"

//protos:
void streamtask(void *param);
int tcpserver(int port);
int http_response(int port, char *req, int connection);
int http_stream(int connection, int convert);
int get_frame(int convert, uint8_t **buf, size_t *len);
static uint16_t set_register(char *uri);
static int set_control(char *uri);
static int get_status(void);
//globals:
camera_fb_t *fb=NULL;
char iobuf[1024]; // for control processing

static const char *TAG = "tcpserver";


/*
starts a task for the streamserver on port 81
then goes into server on port 80 for camera control
*/
void camserver(void)
{
	TaskHandle_t servertask;

    if (!xTaskCreate(&streamtask, "streamserver", 8192, NULL, tskIDLE_PRIORITY+5, &servertask))
    {
        ESP_LOGE(TAG, "***Failed to create stream servertask task");
    }

	tcpserver(80); // server on port 80 to serve camera controls and stills
}

void streamtask(void *param)
{
	tcpserver(81);
}

// main for the tcp webserver custom. This may be a task!
int tcpserver(int port)
{
    char request[400];
    int serverSocket, clientConn, ret,cnt=0;
    struct sockaddr_in IpAddress;  // this is an overlay for the struct sockaddr, that eases the portnumber entry.ie. overlays char sa_data[14] with WORD port, ULONG address
    IpAddress.sin_family = AF_INET;
    IpAddress.sin_port = htons(port); // the port to listen on   **************    this Port ***********************************
    IpAddress.sin_addr.s_addr = INADDR_ANY;//inet_addr("192.168.1.11"); INADDR_ANY, if you dont know it
    socklen_t socklen = sizeof(IpAddress);

    // open internet socket/endpoint for HTTP communication. return file handle or -1=error
    serverSocket = socket(
                       AF_INET,      // Domain: IPv4 Internet protocols
                       SOCK_STREAM,  // Communication-Type:  SOCK_STREAM=TCP; SOCK_DGRAM=UDP
                       IPPROTO_TCP            // was 0: Protocol: 0=IP,internet protocol, pseudo protocol number.TCP and UDP
                   );
    if (serverSocket < 0)
    {
        ESP_LOGE(TAG,"\nsocket failed");
        return -1;
    }


    // assign a specific internet address to the socket. return 0=OK, -1=error
    // normally the local loopback address is assigned. The Port is the one you opened on your router for the machines local LAN address.
    ret=bind(serverSocket, (struct sockaddr *) &IpAddress, socklen );
    if (ret)
    {
        ESP_LOGE(TAG,"\nbind failed");
        return -1;
    }
    // start listening on the socket. returns 0=OK, -1=error
    // The second parameter sets the queue_len for incoming requests.ie. MaxRequests.
    ret = listen(serverSocket, 5);
    if (ret)
    {
        ESP_LOGE(TAG,"\nlisten failed");
        return -1;
    }


   ESP_LOGI(TAG,"Server started on:%s:%u", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port) );

    // wait for connection. We only support 5 connection requests at a time. See listen() above
    // while there are connection requests in the input queue of the socket, process then.
    while(1)
    {
        // wait forever for next tcp connection request from the input queue.
        clientConn = accept(serverSocket, (struct sockaddr *) &IpAddress, &socklen); //this blocks !!
        //printf( "Client connect from: %s:%u\n", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port) );

        // connection is established. loop until closed. So we only allow one connection at a time!
        while (1)
        {
            cnt++;
            // use also poll or select to monitor connection!
            ret = read(clientConn,request,sizeof(request)); //wait for new data on the connection. This blocks!!
            if (ret <= 0)
            {
                //printf("read failed!\n");
                break; // connection lost.  a 0 indicates an orderly disconnect by client; -1 some error occured.
            }
            request[ret]=0;
            //printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Got:\n %sR-EOT\n",request);

            // process response here.....
            ret=http_response(port, request, clientConn);
            //printf("End of Transaction %d <<<<<<<<<<<<<<<<<<<<<<<<\n",cnt);

            // ret=send(clientConn, httpresponse, sizeof(httpresponse), 0); // this may block ???
            if (!ret) break; //close

        }

        //printf("Connection closed\n");
        close(clientConn); // close current tcp connection
    } // endwhile
    return 0;
}


/*
Request-Line = Method SPACE Request-URI SPACE HTTP-Version CRLF
we only support GET requests!
The request URI contains options on which item is requested!
HTTP-Version: always HTTP1.1.

entry: the complete request string
This routine builds and sends the response!
exit: 1= keep connection; 0=drop connection!
*/
const char *resp_index="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nContent-Encoding: gzip\r\n\r\n";
const char *resp_basic="HTTP/1.1 %s\r\n\r\n";
const char *resp_attach="HTTP/1.1 200 OK\r\nContent-Disposition: attachment; filename=\"frame.raw\"\r\nContent-Length: %d\r\n\r\n";
const char *resp_capture="HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nContent-Disposition: inline; filename=capture.jpg\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_status="HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_control="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";

int http_response(int port, char *req, int connection)
{
    // the webpage source is included in the program! This will get the start/end address ..and the length
    extern const unsigned char index_ov2640_html_gz_start[] asm("_binary_index_ov2640_html_gz_start");
    extern const unsigned char index_ov2640_html_gz_end[] asm("_binary_index_ov2640_html_gz_end");
    int indexlength = index_ov2640_html_gz_end - index_ov2640_html_gz_start;
    uint8_t *pb;
    size_t len;
	char response[1024];
	
   char request[10], uri[100];
    int ret,keepalive=1;
    int more=0;
	int freeflag=0;
	uint16_t regval;
    //printf("\n\nLength:%d\n",strlen(req));
    // check for GET: "GET / HTTP/1.1CRLF"
    ret=sscanf(req,"%s %s ",request,uri);
    //printf("req:%s uri:%s ret:%d\n",request,uri,ret);
    if (ret != 2) // some strange request
    {
        sprintf(response,resp_basic,"400 Bad Request");
        goto sendresponse;
    }

    if (strcmp(request,"GET")) // not a GET request
    {
        sprintf(response,resp_basic,"501 Not Implemented");
        goto sendresponse;
    }
	
if (port == 80) // control port
{	
    //we are now at GET
    if (!strcmp(uri,"/")||!strcmp(uri,"/index.html")) // request for index.html
    {
        // send the webpage
        sprintf(response,resp_index,indexlength);
        pb=(uint8_t*)index_ov2640_html_gz_start;
        len=indexlength;
        goto sendmore;
    }


    // send status
    if (!strcmp(uri,"/status"))
    {
        if (!get_status()) goto send500;
        sprintf(response,resp_status,strlen(iobuf));
        strcat(response,iobuf);
        goto sendresponse;
    }


    // set control
    if (!strncmp(uri,"/control",8))
    {
        //printf("+++++++++++++control\n");
        ret = set_control(uri);
//        if (ret != 1) goto send404;  webpage freezes if 404 is returned, so dont do it
        sprintf(response,resp_control,0);
        goto sendresponse;
    }

	// Set/Get register value
    if ( !strncmp(uri,"/reg",4) ) // set a register
    {
		regval=set_register(uri);
        sprintf(response,resp_control,0);
        goto sendresponse;
		
	}
    if ( !strncmp(uri,"/greg",5) ) // get a register
    {
		regval=set_register(uri);
		sprintf(iobuf,"%u",regval);
        sprintf(response,resp_status,strlen(iobuf));
        strcat(response,iobuf);
        goto sendresponse;
	}


    // download raw image!!
    if (!strcmp(uri,"/download"))
    {
		freeflag=1;
        if (!get_frame(0,&pb,&len))
            goto send500;
        else
        {
            sprintf(response,resp_attach,len);
            goto sendmore;
        }

    }

    // capture image!!
    if (!strncmp(uri,"/capture",8))
    {
		freeflag=1;
         if (!get_frame(0,&pb,&len))
            goto send500;
        else
        {
            //printf("--pbuf:0x%08x len:%d\n",(uint32_t)pb,len);
            sprintf(response,resp_capture,len);

            goto sendmore;
        }

    }
}
if (port == 81)
{
    // http stream
    if (!strncmp(uri,"/stream",7))
    return(http_stream(connection,0));
}

//default: send 404 not found/supported----this upsets the client as it waits forever,blocks other controls ...maybe just send http ok??!!
//send404:
ESP_LOGE(TAG,"Send404 Unknown GET request: %s",uri);
//    sprintf(response,resp_basic,"404 not found");
        sprintf(response,resp_control,0); // dummy OK

    goto sendresponse;

send500:
    sprintf(response,resp_basic,"500 function-failed");
    goto sendresponse;

sendmore:
    more=1; // send data also
    keepalive=1; // keep connection
sendresponse:
    //printf(">>>>send response:\n%sT-EOT\n",response);
    if (more)
    {
        ret = MSG_MORE;
    }
    else ret = 0;

//debug only prints: D (32360) phy_common: phy_mii_check_link_status(UP)

    ret=send(connection, response, strlen(response),ret);// this blocks until data is sent. ret contains optional MSG_MORE flag to delay sending
    if (ret <= 0)
        return 0; // connection closed. broken connection
    if (ret != strlen(response)) ESP_LOGE(TAG,"send1, not all bytes sent:%d",ret);
    if (more)
    {
        //printf("sending data...\n");
        ret = send(connection, pb, len, 0);// this blocks until data is sent
		if (freeflag) free(pb);
        esp_camera_fb_return(fb);
        fb=NULL;
        if (ret <= 0) // connection closed. broken connection
            return 0;
        if (ret != len) ESP_LOGE(TAG,"send2, not all bytes sent:%d",ret);

    }

    return keepalive;	//in http1.1, always keep connection alive, unless someone hangs up. so always return 1.
}


const char *resp_stream="HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=ESP32CAM_ServerPush\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *frame_header ="\r\n--ESP32CAM_ServerPush\r\nContent-Type:image/jpeg\r\nContent-Length:%d\r\n\r\n"; 
//const char *frame_header ="--ESP32CAM_ServerPush\r\n\r\n";
/* keep a streaming video until remote client hangs up
The content type multipart/x-mixed-replace was developed as part of a technology to emulate server push and streaming over HTTP. 
This implements "The Multipart Content-Type" over HTTP Protocol using boundary-identifier.
This is not to be confused with chunked!!
The identifier can be any string you like;) must stay the same of corse.
MJEPG is working, of corse;)
RAW-YUV results in big data being transmitted, let see. test with firefox.
- convert = 0=format as of camera; 1= convert to ? jpg or bmp in get_frame
returns 0 = close connection
*/
int http_stream(int connection, int convert)
{
	uint8_t *pb;
    size_t len;
char response[512];

	int ret;
	
	// init camera to VGA resolution on start, we dont want too big picture.
	sensor_t *s = esp_camera_sensor_get(); // get the cameras function list
	s->set_framesize(s, FRAMESIZE_VGA);
	//ESP_LOGI(TAG,"Stream Start....");

	ret=send(connection, resp_stream, strlen(resp_stream),0);
	if (ret <= 0) return 0; //client haged up
	
	while (1)
	{
		if (!get_frame(convert,&pb,&len)) break;
		
		sprintf(response,frame_header);
		ret=send(connection, response, strlen(response),0);
		if (ret <= 0) //connection closed by client
		{
			if (convert) free(pb);
			break;
		}
		ret = send(connection, pb, len, 0);// this blocks until data is sent
		 if (convert) free(pb); // jpg conversion buffer
        esp_camera_fb_return(fb);
        fb=NULL;
 
		if (ret <= 0) break; //connection closed by client
        else if (ret != len) ESP_LOGE(TAG,"sendjpg, not all bytes sent:%d errno:%d",ret,errno);
	}
//ESP_LOGI(TAG,"....Stream Stop");
	
	return 0; // hangup
}


/*
get a frame from the camera and optionally convert to jpg
uses global pointer to empty fb_struct
entry:
- covert flag, if 1, convert to jpg
- address of pointer to buffer containing result data
- address of len variable receiving the length of data
exit:
1=OK, 0=capture or convert failed.
The buffer with data and the length is returned to caller using pointers!!
*/
int get_frame(int convert, uint8_t **buf, size_t *len)
{
    esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG,"CamCapture failed");
        return 0;
    }
    else
    {
        *buf=fb->buf;
        *len=fb->len;
    }
    if (convert)
    {
        //printf("\nstart jpg convert(%u)...\n",*len);
        if (frame2jpg(fb, 80, buf, len) != 1)
        {
            //printf("jpg-convert failed\n");
            return 0;
        }
        //printf("...convert done(%u)!\n",*len);

    }
    return 1;
}


/* process a register get/set command from server:
entry:
- uri-string containing the json formatted request: fe. URI: "/reg?reg=12296&mask=255&val=2" or "/greg?reg=12296&mask=255"
exit:
  the value returned or 0=OK. no error checking!!!
*/
static uint16_t set_register(char *uri)
{
    char *pfunction, *preg, *pvalue;
    int value=0,reg,setflag=0;
    sensor_t *s;
    // get json parameters from uri
	uri++; //skip leading /
    pfunction=strtok(uri, "?"); //returns "reg" or "greg"
	strtok(NULL, "=");//returns: "reg"
    preg=strtok(NULL, "&");//  returns: "12296"
	reg=atoi(preg);
    strtok(NULL, "="); // returns: "mask"
	if (!strcmp(pfunction,"reg"))
	{
		setflag=11; // we are setting a register
		strtok(NULL, "="); // returns: "255&val"
		pvalue=strtok(NULL, "="); // returns: "2". didnt find '=' but rturns the last string
		value=atoi(pvalue);
	}
   ESP_LOGI(TAG, "Register: %s reg=0x%02x value:0x%02x", pfunction, reg, value);
   
       s = esp_camera_sensor_get(); // get the cameras function list

   if (setflag)
   {
	   if (s->set_reg) return(s->set_reg(s,reg,0xff, value)); // set register with value
   }
   else
   {
	   if (s->get_reg) return(s->get_reg(s,reg,0xff)); // set register with value
   }
ESP_LOGI(TAG, "register function not supported!");   
return 0;   
}


/* process a set control command from server:
entry:
- uri-string containing the json formatted request: fe. URI: /control?var=face_detect&val=0
exit:
- 1 = OK
- 0 = fail, control not found. 
-1 = set function failed. 
*/
static int set_control(char *uri)
{
    char *variable, *ps;
    int value;
    sensor_t *s;
    int  (*func)(sensor_t *sensor, int val)=NULL;
    // get json parameters from uri
    strtok(uri, "=&"); //goto first & or = .tell strtok to use string uri. returns: "/control?var"
    variable=strtok(NULL, "=&");// from last = find next = or & and put a /0 there. returns: "face_detect"
    strtok(NULL, "="); // returns: "val"
    ps=strtok(NULL, "="); // returns: "0". didnt find '=' but rturns the last string
    value=atoi(ps);
    ESP_LOGI(TAG, "Control: %s = %d", variable, value);


    s = esp_camera_sensor_get(); // get the cameras function list

    if (!strcmp(variable, "framesize")) func=(void*)s->set_framesize;
    else if (!strcmp(variable, "contrast")) func = s->set_contrast;
    else if (!strcmp(variable, "brightness")) func = s->set_brightness;
    else if (!strcmp(variable, "saturation")) func = s->set_saturation;
    else if (!strcmp(variable, "gainceiling")) func = (void*)s->set_gainceiling;
    else if (!strcmp(variable, "colorbar")) func = s->set_colorbar;
    else if (!strcmp(variable, "awb")) func = s->set_whitebal;
    else if (!strcmp(variable, "agc")) func = s->set_gain_ctrl;
    else if (!strcmp(variable, "aec")) func = s->set_exposure_ctrl;
    else if (!strcmp(variable, "hmirror")) func = s->set_hmirror;
    else if (!strcmp(variable, "vflip")) func = s->set_vflip;
    else if (!strcmp(variable, "awb_gain")) func = s->set_awb_gain;
    else if (!strcmp(variable, "agc_gain")) func = s->set_agc_gain;
    else if (!strcmp(variable, "aec_value")) func = s->set_aec_value;
    else if (!strcmp(variable, "aec2")) func = s->set_aec2;
    else if (!strcmp(variable, "dcw")) func = s->set_dcw;
    else if (!strcmp(variable, "bpc")) func = s->set_bpc;
    else if (!strcmp(variable, "wpc")) func = s->set_wpc;
    else if (!strcmp(variable, "raw_gma")) func = s->set_raw_gma;
    else if (!strcmp(variable, "lenc")) func = s->set_lenc;
    else if (!strcmp(variable, "special_effect")) func = s->set_special_effect;
    else if (!strcmp(variable, "wb_mode")) func = s->set_wb_mode;
    else if (!strcmp(variable, "ae_level")) func = s->set_ae_level;

    if (func == NULL)
    {
        ESP_LOGI(TAG,"Control not supported");
        return 0; //setting not supported by camera
    }

    if ((*func)(s,value) != 0) return -1; // set value failed


    return 1; // OK
}


/*
get current camera settings status. The actual values are not supported, so all 0
exit:
- 1 = OK
- 0 = fail
*/
static int get_status(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) return 0;
    char *p = iobuf;
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
    p += sprintf(p, ",\"led_intensity\":%d", -1);

    *p++ = '}';
    *p++ = 0;
    return 1;
}