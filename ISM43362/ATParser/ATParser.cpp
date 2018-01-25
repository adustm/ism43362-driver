/* Copyright (c) 2015 ARM Limited
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
 *
 * @section DESCRIPTION
 *
 * Parser for the AT command syntax
 *
 */

#include "ATParser.h"
#include "mbed_debug.h"

#ifdef LF
#undef LF
#define LF  10
#else
#define LF  10
#endif

#ifdef CR
#undef CR
#define CR  13
#else
#define CR  13
#endif
#define MIN(a,b) (((a)<(b))?(a):(b))

#define dbg_on 0

// getc/putc handling with timeouts
int ATParser::putc(char c)
{
    Timer timer;
    timer.start();

    while (true) {
        if (_serial_spi->writeable()) {
            return _serial_spi->putc(c);
        }
        if (timer.read_ms() > _timeout) {
            return -1;
        }
    }
}

int ATParser::getc()
{
    Timer timer;
    timer.start();


    while (true) {
        if (_serial_spi->readable()) {
            return _serial_spi->getc();
        }
        if (timer.read_ms() > _timeout) {
            return -1;
        }
    }
}

void ATParser::flush()
{
    while (_serial_spi->readable()) {
        _serial_spi->getc();
    }
}

// read/write handling with timeouts
int ATParser::write(const char *data, int size_of_data, int size_in_buff)
{
    int i = 0;
    for ( ; i < size_of_data; i++) {
        if (putc(data[i]) < 0) {
            return -1;
        }
    }
    _serial_spi->buffsend(size_of_data + size_in_buff);
    return (size_of_data + size_in_buff);
}

int ATParser::read_withoutnss(char *data, int size)
{
    int totalreadsize = 0;
    int sizetoread = 0;
    int readsize, i,j;
    int nbcalls=1;
    i = 0;

    _bufferMutex.lock();
    this->flush();
    /* Determine the number of call to spi read */
    /* it should not happen as it is handled in ATParser */
    if (size > _buffer_size) { /* several calls to read are required */
        nbcalls = (int) (size / (_buffer_size - 2));
        if (size%(_buffer_size - 2)) {
            nbcalls++;
        }
    }
    
    for (j=0; j< nbcalls; j++) {
        if (j < (nbcalls - 1)) {
            sizetoread = _buffer_size-2;
        } else {
            sizetoread = 8 + size - ((_buffer_size - 2) * j);
        }

        if ( j == 0) {
            readsize = _serial_spi->read_1st(_buffer_size-2);
        } else {
            readsize = _serial_spi->read_no_nss(_buffer_size-2);
        }

        if ( readsize < 0) {
            _serial_spi->disable_nss();
            _bufferMutex.unlock();
            return -1;
        }

        for ( ; i < MIN(readsize, sizetoread); i++) {
            int c = getc();
            if (c < 0) {
                _serial_spi->disable_nss();
                _bufferMutex.unlock();
                return -1;
            }
            data[i] = c;
        }
        totalreadsize += i;
        i = 0;
        if (readsize < sizetoread) {
            break; // no more data to read from the wifi device
        }
    }
    _serial_spi->disable_nss();
    debug_if(dbg_on, "AT< %s\r\n", data);

    _bufferMutex.unlock();

    return (totalreadsize) ;
}

int ATParser::read(char *data)
{
    int readsize;
    int i = 0;

    _bufferMutex.lock();
    this->flush();

    readsize = _serial_spi->read(1200);

    debug_if(dbg_on, "Avail in SPI %d\r\n", readsize);

    if ( readsize < 0) {
        _bufferMutex.unlock();
        return -1;
    }

    for (i = 0 ; i < readsize; i++) {
       int c = getc();
       if (c < 0) {
           _bufferMutex.unlock();
           return -1;
       }
       data[i] = c;
    }

#if TRACE_AT_DATA
    debug_if(dbg_on, "AT<< %d BYTES\r\n", readsize);
    for (i = 0; i < readsize; i++) {
         debug_if(dbg_on, "%2X ", data[i]);
    }
    debug_if(dbg_on, "\r\n");
#endif

    _bufferMutex.unlock();

    return (readsize);
}

// printf/scanf handling
int ATParser::vprintf(const char *format, va_list args)
{
    _bufferMutex.lock();
    if (vsprintf(_buffer, format, args) < 0) {
        _bufferMutex.unlock();
        return false;
    }

    int i = 0;
    for ( ; _buffer[i]; i++) {
        if (putc(_buffer[i]) < 0) {
            _bufferMutex.unlock();
            return -1;
        }
    }
    _bufferMutex.unlock();

    return i;
}

int ATParser::vscanf(const char *format, va_list args)
{
    // Since format is const, we need to copy it into our buffer to
    // add the line's null terminator and clobber value-matches with asterisks.
    //
    // We just use the beginning of the buffer to avoid unnecessary allocations.
    int i = 0;
    int offset = 0;

    _bufferMutex.lock();

    while (format[i]) {
        if (format[i] == '%' && format[i+1] != '%' && format[i+1] != '*') {
            _buffer[offset++] = '%';
            _buffer[offset++] = '*';
            i++;
        } else {
            _buffer[offset++] = format[i++];
        }
    }

    // Scanf has very poor support for catching errors
    // fortunately, we can abuse the %n specifier to determine
    // if the entire string was matched.
    _buffer[offset++] = '%';
    _buffer[offset++] = 'n';
    _buffer[offset++] = 0;

    // To workaround scanf's lack of error reporting, we actually
    // make two passes. One checks the validity with the modified
    // format string that only stores the matched characters (%n).
    // The other reads in the actual matched values.
    //
    // We keep trying the match until we succeed or some other error
    // derails us.
    int j = 0;

    while (true) {
        // Ran out of space
        if (j+1 >= _buffer_size - offset) {
            _bufferMutex.unlock();
            return false;
        }
        // Recieve next character
        int c = getc();
        if (c < 0) {
            _bufferMutex.unlock();
            return -1;
        }
        _buffer[offset + j++] = c;
        _buffer[offset + j] = 0;

        // Check for match
        int count = -1;
        sscanf(_buffer+offset, _buffer, &count);

        // We only succeed if all characters in the response are matched
        if (count == j) {
            // Store the found results
            vsscanf(_buffer+offset, format, args);
            _bufferMutex.unlock();
            return j;
        }
    }
}


// Command parsing with line handling
bool ATParser::vsend(const char *command, va_list args)
{
    int i=0, j=0;
    _bufferMutex.lock();
    // Create and send command
    if (vsprintf(_buffer, command, args) < 0) {
        _bufferMutex.unlock();
        return false;
    }
    /* get buffer length */
    for (i = 0; _buffer[i]; i++) {
    }

    for (j=0; _delimiter[j]; j++) {
        _buffer[i+j] = _delimiter[j];
    }
    _buffer[i+j]=0; // only to get a clean debug log
    
    _serial_spi->buffwrite(_buffer, i+j); /* DEBUG : check returned value */

    debug_if(dbg_on, "AT> %s\n", _buffer);
    _bufferMutex.unlock();
    return true;
}

bool ATParser::vrecv(const char *response, va_list args)
{
    _bufferMutex.lock();
    /* Read from the wifi module, fill _rxbuffer */
    this->flush();
    _serial_spi->read();
restart:
    _aborted = false;
    // Iterate through each line in the expected response
    while (response[0]) {
        // Since response is const, we need to copy it into our buffer to
        // add the line's null terminator and clobber value-matches with asterisks.
        //
        // We just use the beginning of the buffer to avoid unnecessary allocations.
        int i = 0;
        int offset = 0;
        bool whole_line_wanted = false;

        while (response[i]) {
            if (response[i] == '%' && response[i+1] != '%' && response[i+1] != '*') {
                _buffer[offset++] = '%';
                _buffer[offset++] = '*';
                i++;
            } else {
                _buffer[offset++] = response[i++];
                // Find linebreaks, taking care not to be fooled if they're in a %[^\n] conversion specification
                if (response[i - 1] == '\n' && !(i >= 3 && response[i-3] == '[' && response[i-2] == '^')) {
                    whole_line_wanted = true;
                    break;
                }
            }
        }

        // Scanf has very poor support for catching errors
        // fortunately, we can abuse the %n specifier to determine
        // if the entire string was matched.
        _buffer[offset++] = '%';
        _buffer[offset++] = 'n';
        _buffer[offset++] = 0;

        debug_if(dbg_on, "AT? %s\n", _buffer);
        // To workaround scanf's lack of error reporting, we actually
        // make two passes. One checks the validity with the modified
        // format string that only stores the matched characters (%n).
        // The other reads in the actual matched values.
        //
        // We keep trying the match until we succeed or some other error
        // derails us.
        int j = 0;

        while (true) {
            // Recieve next character
            int c = getc();
            if (c < 0) {
                debug_if(dbg_on, "AT(Timeout)\n");
                _bufferMutex.unlock();
                return false;
            }
            // Simplify newlines (borrowed from retarget.cpp)
            if ((c == CR && _in_prev != LF) ||
                (c == LF && _in_prev != CR)) {
                _in_prev = c;
                c = '\n';
            } else if ((c == CR && _in_prev == LF) ||
                       (c == LF && _in_prev == CR)) {
                _in_prev = c;
                // onto next character
                continue;
            } else {
                _in_prev = c;
            }
            _buffer[offset + j++] = c;
            _buffer[offset + j] = 0;

            // Check for oob data
            for (struct oob *oob = _oobs; oob; oob = oob->next) {
                if ((unsigned)j == oob->len && memcmp(
                        oob->prefix, _buffer+offset, oob->len) == 0) {
                    debug_if(dbg_on, "AT! %s\n", oob->prefix);
                    oob->cb();

                    if (_aborted) {
                        debug_if(dbg_on, "AT(Aborted)\n");
                        _bufferMutex.unlock();
                        return false;
                    }
                    // oob may have corrupted non-reentrant buffer,
                    // so we need to set it up again
                    goto restart;
                }
            }

            // Check for match
            int count = -1;
            if (whole_line_wanted && c != '\n') {
                // Don't attempt scanning until we get delimiter if they included it in format
                // This allows recv("Foo: %s\n") to work, and not match with just the first character of a string
                // (scanf does not itself match whitespace in its format string, so \n is not significant to it)
            } else {
                sscanf(_buffer+offset, _buffer, &count);
            }

            // We only succeed if all characters in the response are matched
            if (count == j) {
                debug_if(dbg_on, "AT= %s\n", _buffer + offset);
                // Reuse the front end of the buffer
                memcpy(_buffer, response, i);
                _buffer[i] = 0;

                // Store the found results
                vsscanf(_buffer+offset, _buffer, args);

                // Jump to next line and continue parsing
                response += i;
                break;
            }

            // Clear the buffer when we hit a newline or ran out of space
            // running out of space usually means we ran into binary data
            if ((c == '\n') ) {
                debug_if(dbg_on, "New line AT<<< %s", _buffer+offset);
                j = 0;
            }
            if ((j + 1 >= (_buffer_size - offset))) {

                debug_if(dbg_on, "Out of space AT<<< %s, j=%d", _buffer+offset, j);
                j = 0;
            }
        }
    }

    _bufferMutex.unlock();

    return true;
}


// Mapping to vararg functions
int ATParser::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int res = vprintf(format, args);
    va_end(args);
    return res;
}

int ATParser::scanf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int res = vscanf(format, args);
    va_end(args);
    return res;
}

bool ATParser::send(const char *command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = vsend(command, args);
    va_end(args);
    return res;
}

bool ATParser::recv(const char *response, ...)
{
    va_list args;
    va_start(args, response);
    bool res = vrecv(response, args);
    va_end(args);
    return res;
}


// oob registration
void ATParser::oob(const char *prefix, Callback<void()> cb)
{
    struct oob *oob = new struct oob;
    oob->len = strlen(prefix);
    oob->prefix = prefix;
    oob->cb = cb;
    oob->next = _oobs;
    _oobs = oob;
}

void ATParser::abort()
{
    _aborted = true;
}



