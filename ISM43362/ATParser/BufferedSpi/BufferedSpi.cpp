/**
 * @file    BufferedSpi.cpp
 * @brief   Software Buffer - Extends mbed SPI functionallity
 * @author  Armelle Duboc
 * @version 1.0
 * @see
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

#include "BufferedSpi.h"
#include <stdarg.h>
#include "mbed_debug.h"

// change to true to add few SPI debug lines
#define local_debug false

extern "C" int BufferedPrintfC(void *stream, int size, const char* format, va_list arg);

BufferedSpi::BufferedSpi(PinName mosi, PinName miso, PinName sclk, PinName _nss, PinName _datareadypin,
                         uint32_t buf_size, uint32_t tx_multiple, const char* name)
    : SPI(mosi, miso, sclk, NC), nss(_nss),  _txbuf((uint32_t)(tx_multiple*buf_size)), _rxbuf(buf_size), dataready(_datareadypin)
{
    this->_buf_size = buf_size;
    this->_tx_multiple = tx_multiple;
    this->_sigio_event = 0;
    return;
}

BufferedSpi::~BufferedSpi(void)
{

    return;
}

void BufferedSpi::frequency(int hz)
{
    SPI::frequency(hz);
}

void BufferedSpi::format(int bits, int mode)
{
    SPI::format(bits, mode);
}

void BufferedSpi::disable_nss()
{
    nss = 1;
    wait_us(15);
}

void BufferedSpi::enable_nss()
{
    nss = 0;
    wait_us(15);
}

int BufferedSpi::readable(void)
{
    return _rxbuf.available();  // note: look if things are in the buffer
}

int BufferedSpi::writeable(void)
{
    return 1;   // buffer allows overwriting by design, always true
}

int BufferedSpi::getc(void)
{
    if (_rxbuf.available())
        return _rxbuf;
    else return 0;
}

int BufferedSpi::get16b(void)
{
    int res;
    res = SPI::write(0);  // dummy write to receive
    _rxbuf = (char)(res&0xFF);
    _rxbuf = (char)((res>>8)&0xFF);

    res = _rxbuf;
    res |= ((_rxbuf<<8)&0xFF00);
    return res;
}

int BufferedSpi::putc(int c)
{
    _txbuf = (char)c;

    return c;
}

void BufferedSpi::flush_txbuf(void)
{
    _txbuf.clear();
}

int BufferedSpi::puts(const char *s)
{
    if (s != NULL) {
        const char* ptr = s;
    
        while(*(ptr) != 0) {
            _txbuf = *(ptr++);
        }
        _txbuf = '\n';  // done per puts definition
        BufferedSpi::txIrq();                // only write to hardware in one place
        return (ptr - s) + 1;
    }
    return 0;
}

extern "C" size_t BufferedSpiThunk(void *buf_spi, const void *s, size_t length)
{
    BufferedSpi *buffered_spi = (BufferedSpi *)buf_spi;
    return buffered_spi->buffwrite(s, length);
}

int BufferedSpi::printf(const char* format, ...)
{
    va_list arg;
    va_start(arg, format);
    int r = BufferedPrintfC((void*)this, this->_buf_size, format, arg);
    va_end(arg);
    return r;
}

ssize_t BufferedSpi::buffwrite(const void *s, size_t length)
{
    /* flush buffer from previous message */
    this->flush_txbuf();
    
    /* wait for dataready = 1 */
    while(dataready.read() == 0) {
    }
    this->enable_nss();
    
    if (s != NULL && length > 0) {
        /* 1st fill _txbuf */
        const char* ptr = (const char*)s;
        const char* end = ptr + length;
    
        while (ptr != end) {
            _txbuf = *(ptr++);
        }
        if (length&1) { /* padding to send the last char */
            _txbuf = '\n';
            length++;
        }

        /* 2nd write in SPI */
        BufferedSpi::txIrq();                // only write to hardware in one place
        this->disable_nss();
        return ptr - (const char*)s;
    }
    this->disable_nss();

    return 0;
}

ssize_t BufferedSpi::buffsend(size_t length)
{
    /* wait for dataready = 1 */
    while(dataready.read() == 0) {
    }
    this->enable_nss();

    /* _txbuffer is already filled with data to send */
    /* check if _txbuffer needs padding to send the last char */
    if (length & 1) {
        _txbuf = '\n';
        length++;
    }
    BufferedSpi::txIrq();                // only write to hardware in one place
    this->disable_nss();

    return length;
}

ssize_t BufferedSpi::read()
{
    return this->read(0);
}

ssize_t BufferedSpi::read(int max)
{
    int len = 0;
    int tmp;

    disable_nss();

    /* wait for data ready is up */
    while (dataready.read() == 0) {
        // TO DO handle the timeout
    }

    enable_nss();
    while (dataready.read() == 1) {
        tmp = SPI::write(0);  // dummy write to receive 2 bytes

        if ((tmp&0xFF00) == 0x1500) { // last char reached ?
            wait_us(100);
        }
        if (dataready.read() == 0) { /* end of reception reached */
            if ((tmp&0XFF00) == 0x1500){
                if ((max != 0) && (len < max)) { // to remove once data > buff size is handled
                    _rxbuf = (char)(tmp & 0xFF);
                    len++;
                }
                break;
            }
        }
        if (!((len == 0) && (tmp == 0x0A0D))) {
            /* do not take into account the 2 firts \r \n char in the buffer */
            if ((max == 0) || ((max !=0) && (len < max))) {
                _rxbuf = (char)(tmp & 0x00FF);
                _rxbuf = (char)((tmp >>8)& 0xFF);
                len += 2;
            }
        }
    }
    disable_nss();

    debug_if(local_debug, "SPI READ %d BYTES\r\n", len);

    return len;
}

void BufferedSpi::rxIrq(void)
{
    // read from the peripheral 
    _rxbuf = (char)SPI::write(0);
    if (_cbs[RxIrq]) {
        _cbs[RxIrq]();
    }
    return;
}

void BufferedSpi::txIrq(void)
{ /* write everything available in the _txbuffer */
    int value = 0;
    int dbg_cnt = 0;
    while (_txbuf.available() && (_txbuf.getNbAvailable()>0)) {
        value = _txbuf.get();
        if (_txbuf.available() && ((_txbuf.getNbAvailable()%2)!=0)) {
            value |= ((_txbuf.get()<<8)&0XFF00);
            SPI::write(value);
            dbg_cnt++;
        }
    }
    debug_if(local_debug, "SPI Sent %d BYTES\r\n", 2*dbg_cnt);
    // disable the TX interrupt when there is nothing left to send
    BufferedSpi::attach(NULL, BufferedSpi::TxIrq);
    // trigger callback if necessary
    if (_cbs[TxIrq]) {
        _cbs[TxIrq]();
    }
    return;
}

void BufferedSpi::prime(void)
{
    BufferedSpi::txIrq();                // only write to hardware in one place
    return;
}

void BufferedSpi::attach(Callback<void()> func, IrqType type)
{
    _cbs[type] = func;
}

void BufferedSpi::sigio(Callback<void()> func) {
    core_util_critical_section_enter();
    _sigio_cb = func;
    if (_sigio_cb) {
        if (_sigio_event == 1) {
            _sigio_cb();
            _sigio_event = 0;
        }
    }
    core_util_critical_section_exit();
}

