/* ISM43362 Example
*
* Copyright (c) STMicroelectronics 2017
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string.h>
#include "ISM43362.h"
#include "mbed_debug.h"

// ao activate  / de-activate debug
#define ism_debug false

ISM43362::ISM43362(PinName mosi, PinName miso, PinName sclk, PinName nss, PinName resetpin, PinName datareadypin, PinName wakeup, bool debug)
    : _bufferspi(mosi, miso, sclk, nss, datareadypin), _parser(_bufferspi), _resetpin(resetpin),
      _packets(0), _packets_end(&_packets)
{
    Timer timer;
    uint16_t Prompt[3];
    uint8_t count = 0;
    
    DigitalOut wakeup_pin(wakeup);
    ISM43362::setTimeout((uint32_t)5000);
    _bufferspi.format(16, 0); /* 16bits, ploarity low, phase 1Edge, master mode */
    _bufferspi.frequency(10000000); /* up to 20 MHz */

    _resetpin = 0;
    wait_ms(10);
    _resetpin = 1;
    wait_ms(500);

    _bufferspi.enable_nss();

    timer.start();

    while (_bufferspi.dataready.read() == 1) {
      Prompt[count] =(uint16_t)_bufferspi.get16b();
      count += 1;
      if(timer.read_ms() > 0xFFFF) {
        _bufferspi.disable_nss();
        break;
      }    
    }

    if((Prompt[0] != 0x1515) ||(Prompt[1] != 0x0A0D)||
         (Prompt[2] != 0x203E)) {
      _bufferspi.disable_nss();
    }
    _bufferspi.disable_nss();
    
    _parser.debugOn(debug);
}

/**
  * @brief  Parses and returns number from string.
  * @param  ptr: pointer to string
  * @param  cnt: pointer to the number of parsed digit
  * @retval integer value.
  */
#define CHARISHEXNUM(x)                 (((x) >= '0' && (x) <= '9') || \
                                         ((x) >= 'a' && (x) <= 'f') || \
                                         ((x) >= 'A' && (x) <= 'F'))
#define CHARISNUM(x)                    ((x) >= '0' && (x) <= '9')
#define CHAR2NUM(x)                     ((x) - '0')


extern "C" int32_t ParseNumber(char* ptr, uint8_t* cnt) 
{
    uint8_t minus = 0, i = 0;
    int32_t sum = 0;
    
    if (*ptr == '-') {                                		/* Check for minus character */
        minus = 1;
        ptr++;
        i++;
    }
    while (CHARISNUM(*ptr) || (*ptr=='.')) {   /* Parse number */
        if (*ptr == '.') {
            ptr++; // next char
        } else {
            sum = 10 * sum + CHAR2NUM(*ptr);
            ptr++;
            i++;
        }
    }

    if (cnt != NULL) {                   /* Save number of characters used for number */
        *cnt = i;
    }
    if (minus) {                         /* Minus detected */
        return 0 - sum;
    }
    return sum;                          /* Return number */
}

int ISM43362::get_firmware_version()
{
    if (!(_parser.send("I?") && _parser.recv("ISM43362-M3G-L44-SPI,C3.5.2.3.BETA9,v3.5.2,v1.4.0.rc1,v8.2.1,120000000,Inventek eS-WiFi"))){
        debug("wrong version number\n");
        return -1;
    }
    return (35239);
}

bool ISM43362::startup(int mode)
{
  return false;
}

bool ISM43362::reset(void)
{
  
    _resetpin = 0;
    wait_ms(10);
    _resetpin = 1;
    wait_ms(500);

    return true;
}

bool ISM43362::dhcp(bool enabled)
{
    return (_parser.send("C4=%d", enabled ? 1:0) && _parser.recv("OK"));
}

bool ISM43362::connect(const char *ap, const char *passPhrase)
{
    if (!(_parser.send("C1=%s", ap) && (_parser.recv("OK")))) {
        return false;
    }
    if (!(_parser.send("C2=%s", passPhrase) && (_parser.recv("OK")))) {
        return false;
    }
    /* TODO security level = 3 , is it hardcoded or not ???? */
    if (!(_parser.send("C3=3") && (_parser.recv("OK")))) {
        return false;
    }
    
    /* now connect */
    if (!(_parser.send("C0") && _parser.recv("OK"))) {
        return false;
    }
    return true;
}

bool ISM43362::disconnect(void)
{
    return _parser.send("CD") && _parser.recv("OK");
}

const char *ISM43362::getIPAddress(void)
{
    char tmp_ip_buffer[68] = {0};
    char *ptr, *ptr2;
    if (!_parser.send("C?")) {
        return 0;
    }
    if (!_parser.read(tmp_ip_buffer, sizeof(tmp_ip_buffer)-8)) {
        return 0;
    }

    // Get the IP address in the result
    // TODO : check if the begining of the string is always = "eS-WiFi_AP_C47F51011231,"
    ptr = strtok((char *)tmp_ip_buffer, ",");
    ptr = strtok(NULL, ",");
    ptr = strtok(NULL, ",");
    ptr = strtok(NULL, ",");
    ptr = strtok(NULL, ",");
    ptr = strtok(NULL, ",");
    ptr2 = strtok(NULL, ",");
    if (ptr == NULL) return 0;
    strncpy(_ip_buffer, ptr , ptr2-ptr);

    tmp_ip_buffer[59] = 0;
    debug("receivedIPAddress: %s\n", _ip_buffer);

    return _ip_buffer;
}

const char *ISM43362::getMACAddress(void)
{
    if(!(_parser.send("Z5") && _parser.recv("%s\r\nOK", _mac_buffer))) {
        debug_if(ism_debug,"receivedMacAddress LINE KO: %s", _mac_buffer);
        return 0;
    }

    debug_if(ism_debug,"receivedMacAddress LINE:%s, size=%d\r\n", _mac_buffer, sizeof(_mac_buffer));

    return _mac_buffer;
}

const char *ISM43362::getGateway()
{
    char tmp[250];

    _parser.send("C?");
    int res = _parser.read(tmp, 250);
    if (res <0) {
        debug("receivedGateway: %s", tmp);
        return 0;
    }
    /* Extract the Gateway in the received buffer */

    char *ptr;
    ptr = strtok(tmp,",");
    for (int i = 0; i< 7;i++) {
        if (ptr == NULL) break;
         ptr = strtok(NULL,",");
    }
    
    strncpy(_gateway_buffer, ptr, sizeof(_gateway_buffer));

    return _gateway_buffer;
}

const char *ISM43362::getNetmask()
{
    char tmp[250];
    _parser.send("C?");
    int res = _parser.read(tmp, 250);
    if (res <0) {
        debug("receivedNetmask: %s", tmp);
        return 0;
    }
    
    /* Extract Netmask in the received buffer */
    char *ptr;
    ptr = strtok(tmp,",");
    for (int i = 0; i< 6;i++) {
        if (ptr == NULL) break;
         ptr = strtok(NULL,",");
    }
    
    strncpy(_netmask_buffer, ptr, sizeof(_netmask_buffer));

    return _netmask_buffer;
}

int8_t ISM43362::getRSSI()
{
    int8_t rssi;
    char tmp[25];
    /* Read SSID */
    if (!(_parser.send("CR"))) {
        return 0;
    }
    int res = _parser.read(tmp, 25);
    if (res <0) {
        debug("receivedNetmask: %s", tmp);
        return 0;
    }
    rssi = ParseNumber(tmp+2, NULL);

    return rssi;
}
/**
  * @brief  Parses Security type.
  * @param  ptr: pointer to string
  * @retval Encryption type.
  */
extern "C" nsapi_security_t ParseSecurity(char* ptr) 
{
  if(strstr(ptr,"Open")) return NSAPI_SECURITY_NONE;
  else if(strstr(ptr,"WEP")) return NSAPI_SECURITY_WEP;
  else if(strstr(ptr,"WPA")) return NSAPI_SECURITY_WPA;   
  else if(strstr(ptr,"WPA2 AES")) return NSAPI_SECURITY_WPA2; 
  else if(strstr(ptr,"WPA WPA2")) return NSAPI_SECURITY_WPA_WPA2; 
  else if(strstr(ptr,"WPA2 TKIP")) return NSAPI_SECURITY_UNKNOWN; // ?? no match in mbed ?   
  else return NSAPI_SECURITY_UNKNOWN;           
}

/**
  * @brief  Convert char in Hex format to integer.
  * @param  a: character to convert
  * @retval integer value.
  */
extern "C"  uint8_t Hex2Num(char a) 
{
    if (a >= '0' && a <= '9') {                             /* Char is num */
        return a - '0';
    } else if (a >= 'a' && a <= 'f') {                      /* Char is lowercase character A - Z (hex) */
        return (a - 'a') + 10;
    } else if (a >= 'A' && a <= 'F') {                      /* Char is uppercase character A - Z (hex) */
        return (a - 'A') + 10;
    }
    
    return 0;
}

/**
  * @brief  Extract a hex number from a string.
  * @param  ptr: pointer to string
  * @param  cnt: pointer to the number of parsed digit
  * @retval Hex value.
  */
extern "C" uint32_t ParseHexNumber(char* ptr, uint8_t* cnt) 
{
    uint32_t sum = 0;
    uint8_t i = 0;
    
    while (CHARISHEXNUM(*ptr)) {         /* Parse number */
        sum <<= 4;
        sum += Hex2Num(*ptr);
        ptr++;
        i++;
    }
    
    if (cnt != NULL) {                  /* Save number of characters used for number */
        *cnt = i;
    }
    return sum;                         /* Return number */
}

bool ISM43362::isConnected(void)
{
    return getIPAddress() != 0;
}

int ISM43362::scan(WiFiAccessPoint *res, unsigned limit)
{
    unsigned cnt = 0, num=0;
    nsapi_wifi_ap_t ap;
    char *ptr;
    char tmp[256];

    /* Get the list of AP */
    if (!(_parser.send("F0") && _parser.read_withoutnss(tmp, 256))) {
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    /* Parse the received buffer and fill AP buffer */
    ptr = strtok(tmp + 2, ",");   
  
    while (ptr != NULL) {
        switch (num++) { 
        case 0: /* Ignore index */
        case 4: /* Ignore Max Rate */
        case 5: /* Ignore Network Type */
        case 7: /* Ignore Radio Band */      
            break;
          
        case 1:
            ptr[strlen(ptr) - 1] = 0;
            strncpy((char *)ap.ssid,  ptr+ 1, 32); 
            break;
          
        case 2:
            for (int i=0; i<6; i++) {
                ap.bssid[i] = ParseHexNumber(ptr + (i*3), NULL);
            }
            break;

        case 3: 
            ap.rssi = ParseNumber(ptr, NULL);
            break;
          
        case 6: 
            ap.security = ParseSecurity(ptr);
            break;      

        case 8:            
            ap.channel = ParseNumber(ptr, NULL);
            res[cnt] = WiFiAccessPoint(ap);
            cnt++; 
            num = 1;
            break;

        default: 
            break;
        }
        ptr = strtok(NULL, ",");
        if (limit != 0 && cnt >= limit) {
            break;
        }
    }  

    return cnt;
}

bool ISM43362::open(const char *type, int id, const char* addr, int port)
{ /* TODO : This is the implementation for the client socket, need to check if need to create openserver too */
    //IDs only 0-3
    if((id < 0) ||(id > 3)) {
        debug("open: wrong id\n");
        return false;
    }
    /* Set communication socket */
    if (!(_parser.send("P0=%d", id) && _parser.recv("OK"))) {
        return false;
    }
    /* Set protocol */
    if (!(_parser.send("P1=%s", type) && _parser.recv("OK"))) {
        return false;
    }
    /* Set address */
    if (!(_parser.send("P3=%s", addr) && _parser.recv("OK"))) {
        return false;
    }
    if (!(_parser.send("P4=%d", port) && _parser.recv("OK"))) {
        return false;
    }
    /* Start client */
    if (!(_parser.send("P6=1") && _parser.recv("OK"))) {
        return false;
    }
    return true;
}

bool ISM43362::dns_lookup(const char* name, char* ip)
{
    char tmp[30];
    char *ptr;
    if (!(_parser.send("D0=%s", name) && _parser.read(tmp,30))) {
        return 1;
    }
    ptr = strchr(tmp,'\r');
    strncpy(ip, tmp, (int)(ptr - tmp));
    *(ip + (ptr - tmp)) = 0;
    debug("ip of DNSlookup: %s\n", ip);
    return 1;
}

bool ISM43362::send(int id, const void *data, uint32_t amount)
{
    // TODO CHECK SIZE NOT > txbuf size
    debug_if(ism_debug, "ISM43362 send%d\r\n", amount);
    /* Activate the socket id in the wifi module */
    if ((id < 0) ||(id > 3)) {
        return false;
    }
    if (!(_parser.send("P0=%d",id) && _parser.recv("OK"))) {
        return false;
    }
    /* Change the write timeout */
    if (!(_parser.send("S2=%d", _timeout) && _parser.recv("OK"))) {
        return false;
    }
    /* set Write Transport Packet Size */
    int i = _parser.printf("S3=%d\r", (int)amount);
    if (i < 0) {
        return false;
    }
    i = _parser.write((const char *)data, amount, i);
    if (i < 0) {
        return false;
    }

    if (!_parser.recv("OK")) {
        return false;
    }

    return true;
}

void ISM43362::_packet_handler()
{
    int id;
    uint32_t amount;

    debug_if(ism_debug, "ISM43362 _packet_handler\r\n");

    // parse out the packet
    if (!_parser.recv(",%d,%d:", &id, &amount)) {
        return;
    }

    struct packet *packet = (struct packet*)malloc(
            sizeof(struct packet) + amount);
    if (!packet) {
        return;
    }

    packet->id = id;
    packet->len = amount;
    packet->next = 0;

    if (!(_parser.read((char*)(packet + 1), amount))) {
        free(packet);
        return;
    }
    debug_if(ism_debug, "%d BYTES\r\n", amount);


    // append to packet list
    *_packets_end = packet;
    _packets_end = &packet->next;
}

int32_t ISM43362::recv(int id, void *data, uint32_t amount)
{
    debug_if(ism_debug, "ISM43362 req recv=%d\r\n", amount);

    /* Activate the socket id in the wifi module */
    if ((id < 0) ||(id > 3)) {
        return false;
    }
    if (!(_parser.send("P0=%d",id) && _parser.recv("OK"))) {
        return false;
    }

    /* Change receive timeout */
    if (!(_parser.send("R2=%d", _timeout) && _parser.recv("OK"))) {
        return false;
    }
    int nbcalls = (amount / (_parser.get_size() - 2));
    if (amount %( _parser.get_size() - 2)) {
        nbcalls++;
    }
    /* Need to handle several calls to the Read Transport command in case
       the amount to read is longer than the buffer size */
    int i, read_amount = 0;
    int total_read = 0, amount_to_read;
    for (i=0; i < nbcalls; i++) {
        if ((nbcalls > 1) && ( i != (nbcalls - 1))) {
            amount_to_read = _parser.get_size() - 2;
        } else {
            amount_to_read = amount - (i * (_parser.get_size() - 2));
        }
        /* Set the amount of datas to read */
        if (!(_parser.send("R1=%d", amount_to_read) && _parser.recv("OK"))) {
            return -1;
        }
        /* Now read */
        if (!_parser.send("R0")) {
            return -1;
        }
        read_amount = _parser.read((char *)((uint32_t)data+total_read), amount_to_read);
        if (read_amount <0) {
            return -1;
        }

        if (strncmp("OK\r\n> ", (char *)data, 6) == 0) {
            debug_if(ism_debug, "ISM4336 recv 2nothing to read=%d\r\n", read_amount);
            return 0; /* nothing to read */
        }

        /* reception is complete: remove the last 8 chars that are : \r\nOK\r\n> */
        total_read += read_amount;
        /* bypass ""\r\nOK\r\n> " if present at the end of the chain */
        if ((total_read >= 8) && (strncmp((char *)((uint32_t) data + total_read - 8), "\r\nOK\r\n> ", 8)) == 0) {
            total_read -= 8;
        }
    }

    debug_if(ism_debug, "ISM43362 total_read=%d\r\n", total_read);

    return total_read;
}

int ISM43362::check_recv_status(int id, void *data, uint32_t amount)
{
    uint32_t read_amount;
    debug_if(ism_debug, "ISM43362 req check_recv_status=%d\r\n", amount);
    /* Activate the socket id in the wifi module */
    if ((id < 0) ||(id > 3)) {
        return -1;
    }
    if (!(_parser.send("P0=%d",id) && _parser.recv("OK"))) {
        return -1;
    }

    /* Change receive timeout */
    if (!(_parser.send("R2=%d", _timeout) && _parser.recv("OK"))) {
        return -1;
    }
    /* Read if data is = "OK\r\n" -> nothing to read, or if data is <> meaining something to read */
    if (!(_parser.send("R1=%d", amount)&& _parser.recv("OK"))) {
            return -1;
    }
    if (!_parser.send("R0")) {
        return -1;
    }
    read_amount = _parser.read((char *)data, amount);
    if (strncmp("OK\r\n> ", (char *)data, 6) == 0) {
        debug_if(ism_debug, "ISM4336 recv 2 nothing to read=%d\r\n", read_amount);
        return 0; /* nothing to read */
    } else {
        /* bypass ""\r\nOK\r\n> " if present at the end of the chain */
        if ((read_amount >= 8) && (strncmp((char *)((uint32_t) data + read_amount - 8), "\r\nOK\r\n> ", 8)) == 0) {
            read_amount -= 8;
        }
        debug_if(ism_debug, "ISM43362 read_amount=%d\r\n", read_amount);
        return read_amount;
    }
}

bool ISM43362::close(int id)
{
    if ((id <0) || (id > 3)) {
        debug("Wrong socket number\n");
        return false;
    }
    /* Set connection on this socket */
    if (!(_parser.send("P0=%d", id) && _parser.recv("OK"))){
        return false;
    }
    /* close this socket */
    if (!(_parser.send("P6=0") && _parser.recv("OK"))){
        return false;
    }
    return true;
}

void ISM43362::setTimeout(uint32_t timeout_ms)
{
    _timeout = timeout_ms;
    _parser.setTimeout(timeout_ms);
}

bool ISM43362::readable()
{
  /* not applicable with SPI api */
    return true;
}

bool ISM43362::writeable()
{
    /* not applicable with SPI api */
    return true;
}

void ISM43362::attach(Callback<void()> func)
{
    /* not applicable with SPI api */
}

bool ISM43362::recv_ap(nsapi_wifi_ap_t *ap)
{
    int sec = 0;
    char tmp[350];
    int ret = _parser.read(tmp, 350);
    if (ret != -1) {
        /* TODO fill networkaccess points */
        if (ap != NULL) {
            ap->security = sec < 5 ? (nsapi_security_t)sec : NSAPI_SECURITY_UNKNOWN;
        }
        return true;
    }
    return false;
}

void ISM43362::reset_module(DigitalOut rstpin)
{
    rstpin = 0;
    wait_ms(10);
    rstpin = 1;
    wait_ms(500);
}

