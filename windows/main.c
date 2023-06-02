#include <time.h>
#include <stdint.h>
#include <winsock.h>
#include <errno.h>
#include <stdio.h>
#include <assert.h>
#include "modbus_constants.h"
#include <io.h>


#include <fcntl.h>

typedef struct timeval time_val;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint16_t u16;
typedef int32_t  s32;
typedef int64_t  s64;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint8_t  u8;
typedef s8       b8;
typedef u32      ssize_t;


#define function               static
#define global_variable        static
#define local_persist          static

typedef struct _sft {
	u32 slave;
	u32 func;
	u32 t_id;
}sft_t;

#define PY_BUF_SIZE 512

typedef struct win32_ser {
	//File handle
	HANDLE fd;
	//Received buffer
	u8 buf[PY_BUF_SIZE];
	//Received chars
	DWORD n_byts;
}win32_ser;

typedef struct {
	/* device : "/dev/ttyS0", "/dev/ttyUSB0" etc */
	/* baud : 9600, 19200, 57600, 115200, etc */
	/* DCB : win32, defines the settings for serial communication device*/

	char* device;
	s32       baud;
	u8        data_bit;
	u8        stop_bit;
	char      parity;
	win32_ser w_ser;
	DCB       old_dcb;

	//To handle many slaves on the same link
	s32       confirmation_to_ignore;
}modbus_rtu;

typedef enum {
	_STEP_FUNCTION,
	_STEP_META,
	_STEP_DATA
}_step_t;


typedef struct {
	s32 slave;
	s32 s;
	s32 debug;
	s32 error_recovery;
	s32 quirks;
	time_val response_timeout;
	time_val byte_timeout;
	time_val indication_timeout;

	//if this is a rtu
	//TODO: add modbus_tcp
	modbus_rtu* rtu_handle;
}modbus_t;

/*
	CRC stands for Cyclic Reduncany check. 
	CRC is two bytes added to the end of each modbus message for error checking. 
	Each byte in the message is sent to calculate the CRC. 
	The receiving device also calculates the CRC and compares it to the CRC from the sending device.
*/

function u16 
crc16(u8* buffer, u16 buffer_length) {
	u8 crc_hi = 0xFF;
	u8 crc_lo = 0xFF;

	u32 i;

	while (buffer_length--) {
		i = crc_lo ^ *buffer++;
		crc_lo = crc_hi ^ table_crc_hi[i];
		crc_hi = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

const char *modbus_strerror(int errnum)
{
    switch (errnum) {
    case EMBXILFUN:
        return "Illegal function";
    case EMBXILADD:
        return "Illegal data address";
    case EMBXILVAL:
        return "Illegal data value";
    case EMBXSFAIL:
        return "Slave device or server failure";
    case EMBXACK:
        return "Acknowledge";
    case EMBXSBUSY:
        return "Slave device or server is busy";
    case EMBXNACK:
        return "Negative acknowledge";
    case EMBXMEMPAR:
        return "Memory parity error";
    case EMBXGPATH:
        return "Gateway path unavailable";
    case EMBXGTAR:
        return "Target device failed to respond";
    case EMBBADCRC:
        return "Invalid CRC";
    case EMBBADDATA:
        return "Invalid data";
    case EMBBADEXC:
        return "Invalid exception code";
    case EMBMDATA:
        return "Too many data";
    case EMBBADSLAVE:
        return "Response not from requested slave";
    default:
        return strerror(errnum);
    }
}

function void 
_error_print(modbus_t *ctx, const char *context)
{
    if (ctx->debug) {
        fprintf(stderr, "ERROR %s", modbus_strerror(errno));
        if (context != NULL) {
            fprintf(stderr, ": %s\n", context);
        } else {
            fprintf(stderr, "\n");
        }
    }
}

#pragma pack(push, 1)
typedef union {
	struct {
		u8 slave;
		u8 func;
		u8 addr_1;
		u8 addr_2;
		u8 nb_1;
		u8 nb_2;
	};
	u8 raw[6];
}req_raw;

#pragma pack(pop, 1)

/*
	* Builds a RTU request header
	* slave, function, address, number of bytes
*/
function s32
rtu_build_request_basis(modbus_t* ctx, s32 func, s32 addr, s32 nb, u8* req) {
	assert(ctx->slave != -1);
	assert(req != NULL);
	if (req != NULL) {
		req[0] = ctx->slave;
		req[1] = func;
		req[2] = addr >> 8;
		req[3] = addr & 0x00ff;
		req[4] = nb >> 8;
		req[5] = nb & 0x00ff;
	}

	req_raw* request = (req_raw*)req;
	return _RTU_PRESET_REQ_LENGTH;
}

function s32
rtu_build_response_basis(sft_t* sft, u8* rsp) {
	rsp[0] = sft->slave;
	rsp[1] = sft->func;
	return _RTU_PRESET_RSP_LENGTH;
}

function s32
modbus_set_slave(modbus_t* ctx, s32 slave) {
	s32 max_slave = (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;

	if (slave >= 0 && slave <= max_slave) {
		ctx->slave = slave;
	}
	else {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

function void
_modbus_init_common(modbus_t* ctx) {
	ctx->slave = -1;
	ctx->s = -1;
	ctx->debug = FALSE;
	ctx->error_recovery = 0;
	ctx->quirks = MODBUS_QUIRK_NONE;
	ctx->response_timeout.tv_sec = 0;
	ctx->response_timeout.tv_usec = _RESPONSE_TIMEOUT;

	ctx->byte_timeout.tv_sec = 0;
	ctx->byte_timeout.tv_usec = _BYTE_TIMEOUT;

	ctx->indication_timeout.tv_sec = 0;
	ctx->indication_timeout.tv_usec = 0;
}

function void
modbus_free(modbus_t* ctx) {
	if (ctx) {
		if (ctx->rtu_handle) {
			free(ctx->rtu_handle);
		}
		free(ctx);
	}
}

function b8 
is_connected(modbus_t* ctx){
	return ctx->rtu_handle->w_ser.fd != INVALID_HANDLE_VALUE;
}

function void 
_sleep_response_timeout(modbus_t* ctx) {
	Sleep((ctx->response_timeout.tv_sec * 1000) + (ctx->response_timeout.tv_usec / 1000));
}

function void
rtu_close(modbus_t* ctx) {
	modbus_rtu* ctx_rtu = ctx->rtu_handle;

	if (!SetCommState(ctx_rtu->w_ser.fd, &ctx_rtu->old_dcb) && ctx->debug) {
		fprintf(stderr, "Error couldn't revert to configuration (LastError %d) \n", (int)GetLastError());
	}

	if (!CloseHandle(ctx_rtu->w_ser.fd) && ctx->debug) {
		fprintf(stderr, "Error while closing handle (LastError %d) \n", (int)GetLastError());
	}
}

function s32 
win32_ser_select(struct win32_ser *ws, int max_len, const struct timeval *tv)
{
    COMMTIMEOUTS comm_to;
    unsigned int msec = 0;

    /* Check if some data still in the buffer to be consumed */
    if (ws->n_byts > 0) {
        return 1;
    }

    /* Setup timeouts like select() would do.
       FIXME Please someone on Windows can look at this?
       Does it possible to use WaitCommEvent?
       When tv is NULL, MAXDWORD isn't infinite!
     */
    if (tv == NULL) {
        msec = MAXDWORD;
    } else {
        msec = tv->tv_sec * 1000 + tv->tv_usec / 1000;
        if (msec < 1)
            msec = 1;
    }

    comm_to.ReadIntervalTimeout = msec;
    comm_to.ReadTotalTimeoutMultiplier = 0;
    comm_to.ReadTotalTimeoutConstant = msec;
    comm_to.WriteTotalTimeoutMultiplier = 0;
    comm_to.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(ws->fd, &comm_to);

    /* Read some bytes */
    if ((max_len > PY_BUF_SIZE) || (max_len < 0)) {
        max_len = PY_BUF_SIZE;
    }

    if (ReadFile(ws->fd, &ws->buf, max_len, &ws->n_byts, NULL)) {
        /* Check if some bytes available */
        if (ws->n_byts > 0) {
            /* Some bytes read */
            return 1;
        } else {
            /* Just timed out */
            return 0;
        }
    } else {
        /* Some kind of error */
        return -1;
    }
}

function s32 
rtu_flush(modbus_t* ctx) {
	modbus_rtu* ctx_rtu    = ctx->rtu_handle;
	ctx_rtu->w_ser.n_byts = 0;
	return (PurgeComm(ctx_rtu->w_ser.fd, PURGE_RXCLEAR) == FALSE);
}

function ssize_t 
rtu_recv(modbus_t* ctx, u8* rsp, u32 rsp_length) {
	u32 n = ctx->rtu_handle->w_ser.n_byts;

	if (rsp_length < n) {
		n = rsp_length;
	}

	if (n > 0) {
		memcpy(rsp, ctx->rtu_handle->w_ser.buf, n);
	}
	ctx->rtu_handle->w_ser.n_byts -= n;
	return n;
}

function u32
compute_meta_length_after_function(s32 func, msg_type_t msg_type) {

	u32 length;

	if (msg_type == MSG_INDICATION) {
		if (func <= MODBUS_FC_WRITE_SINGLE_REGISTER) {
			length = 4;
		}
		else if (func == MODBUS_FC_WRITE_MULTIPLE_COILS ||
			func == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
			length = 5;
		}
		else if (func == MODBUS_FC_MASK_WRITE_REGISTER) {
			length = 6;
		}
		else if (func == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
			length = 9;
		}
		else {
			/* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
			length = 0;
		}
	}
	else {
		/* MSG_CONFIRMATION */
		switch (func) {
		case MODBUS_FC_WRITE_SINGLE_COIL:
		case MODBUS_FC_WRITE_SINGLE_REGISTER:
		case MODBUS_FC_WRITE_MULTIPLE_COILS:
		case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
			length = 4;
			break;
		case MODBUS_FC_MASK_WRITE_REGISTER:
			length = 6;
			break;
		default:
			length = 1;
		}
	}
	return length;
}

function s32
compute_meta_length_after_meta(modbus_t* ctx, u8* msg, msg_type_t msg_type) {
	s32 func = msg[_MODBUS_RTU_HEADER_LENGTH];
	s32 len;
    if (msg_type == MSG_INDICATION) {
        switch (func) {
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            len = msg[_MODBUS_RTU_HEADER_LENGTH + 5];
            break;
        case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            len = msg[_MODBUS_RTU_HEADER_LENGTH + 9];
            break;
        default:
            len = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        if (func <= MODBUS_FC_READ_INPUT_REGISTERS ||
            func == MODBUS_FC_REPORT_SLAVE_ID ||
            func == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            len = msg[_MODBUS_RTU_HEADER_LENGTH + 1];
        } else {
            len = 0;
        }
    }

    len += _MODBUS_RTU_CHECKSUM_LENGTH;
    return len;
}

function s32
rtu_check_integrity(modbus_t* ctx, u8* msg, const s32 msg_length) {
	u16 crc_calculated;
	u16 crc_received;
	s32 slave = msg[0];

	if (slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
		if (ctx->debug) {
			printf("Request for slave %d ignored (not %d)\n", slave, ctx->slave);
		}
		return 0;
	}

	crc_calculated = crc16(msg, msg_length - 2);
	crc_received = (msg[msg_length - 1] << 8) | msg[msg_length - 2];

	if (crc_calculated == crc_received) {
		return msg_length;
	}
	else {
		if (ctx->debug) {
			fprintf(stderr, "ERROR CRC recived 0x%0X != CRC calculated 0x%0X \n", crc_received, crc_calculated);
		}

		if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			rtu_flush(ctx);
		}
		errno = EMBBADCRC;
		return -1;
	}
}

function void win32_ser_init(win32_ser* ws) {
	memset(ws, 0x00, sizeof(win32_ser));
	ws->fd = INVALID_HANDLE_VALUE;
}

function s32 rtu_connect(modbus_t* ctx) {
	DCB dcb;
	assert(ctx && ctx->rtu_handle);
	if (ctx->rtu_handle != NULL) {
		modbus_rtu* rtu = ctx->rtu_handle;

		if (ctx->debug) {
			printf("Opening %s at %d bauds (%c, %d, %d)\n",
				rtu->device,
				rtu->baud,
				rtu->parity,
				rtu->data_bit,
				rtu->stop_bit);
		}
		win32_ser_init(&rtu->w_ser);

		/*
			rtu->device should contain a string like "COMxx:" xx being decimal number
		*/

		rtu->w_ser.fd = CreateFileA(rtu->device, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
		//ctx->s = _open(rtu->device, _O_RDWR);

		if (rtu->w_ser.fd == INVALID_HANDLE_VALUE) {
			if (ctx->debug) {
				fprintf(stderr, "ERROR can't open the device %s (LastError %d)\n", rtu->device, (int)GetLastError());
			}
			return -1;
		}

		rtu->old_dcb.DCBlength = sizeof(DCB);

		if (!GetCommState(rtu->w_ser.fd, &rtu->old_dcb)) {
			if (ctx->debug) {
				fprintf(stderr, "Error getting configurateion (LastError %d)\n", (int)GetLastError());
			}
			CloseHandle(rtu->w_ser.fd);
			rtu->w_ser.fd = INVALID_HANDLE_VALUE;
			return -1;
		}
		//Build new configuration starting from current settings
		dcb = rtu->old_dcb;
		dcb.BaudRate = rtu->baud;
		dcb.ByteSize = rtu->data_bit;

		if (rtu->stop_bit == 1)  dcb.StopBits = ONESTOPBIT;
		else dcb.StopBits = TWOSTOPBITS;

		if (rtu->parity == 'N') {
			dcb.Parity = NOPARITY;
			dcb.fParity = FALSE;
		}
		else if (rtu->parity == 'E') {
			dcb.Parity = EVENPARITY;
			dcb.fParity = TRUE;
		}
		else {
			dcb.Parity = ODDPARITY;
			dcb.fParity = TRUE;
		}

		//NO software handshake
		dcb.fTXContinueOnXoff = TRUE;
		dcb.fOutX = FALSE;
		dcb.fInX = FALSE;

		dcb.fBinary = TRUE;
		dcb.fAbortOnError = FALSE;

		if (!SetCommState(rtu->w_ser.fd, &dcb)) {
			if (ctx->debug) {
				fprintf(stderr, "Error setttings new configuration (LastError) %d", (int)GetLastError());
			}

			CloseHandle(rtu->w_ser.fd);
			rtu->w_ser.fd = INVALID_HANDLE_VALUE;
			return -1;
		}
	}
	return 0;
}
function s32 
rtu_select(modbus_t* ctx, fd_set* rset, time_val* tv, s32 length_to_read) {
	int s_rc;

	s_rc = win32_ser_select(&ctx->rtu_handle->w_ser, length_to_read, tv);

	if (s_rc == 0) {
		errno = ETIMEDOUT;
		return -1;
	}
	if (s_rc < 0) {
		return -1;
	}

#if 0
	while ((s_rc = select( NULL , rset, NULL, NULL, tv)) == -1){
		if (errno == EINTR) {
			if (ctx->debug) {
				fprintf(stderr, "A non blocked signal was caught \n");
			}
			FD_ZERO(rset);
			FD_SET(ctx->s, rset);
		}
		else {
			return -1;
		}
	}

	if (s_rc == 0) {
		errno = ETIMEDOUT;
		return -1;
	}
#endif
	return s_rc;
}


function s32
rtu_receive_message(modbus_t* ctx, u8* msg, msg_type_t msg_type) {
	s32 rc;
	fd_set rset;
	time_val tv;
	time_val* p_tv;
	u32 length_to_read;
	s32 msg_length = 0;
	_step_t step;

	s32 wsa_err;

	if (ctx->debug) {
		if (msg_type == MSG_INDICATION) {
			printf("Waiting for an indication ... \n");
		}
		else {
			printf("Waiting for a confirmation ... \n");
		}
	}

	if (!is_connected(ctx)) {
		if (ctx->debug) {
			printf("Error the connection is not established .\n");
		}
		return -1;
	}
	// int fd = _open_osfhandle((long)ctx->rtu_handle->w_ser.fd, O_RDWR |  O_EXCL );
	FD_ZERO(&rset);
	//FD_SET(fd , &rset);

	/**
	 * Analyse the message step by step
	 */
	step           = _STEP_FUNCTION;
	length_to_read = _MODBUS_RTU_HEADER_LENGTH + 1;

	if (msg_type == MSG_INDICATION) {
		if (ctx->indication_timeout.tv_sec == 0 && ctx->indication_timeout.tv_usec == 0) {
			p_tv = NULL;
		}
		else {
			tv.tv_usec = ctx->indication_timeout.tv_usec;
			tv.tv_sec  = ctx->indication_timeout.tv_sec;
			p_tv = &tv;
		}
	}
	else {
		tv.tv_usec = ctx->response_timeout.tv_usec;
		tv.tv_sec  = ctx->response_timeout.tv_sec;
		p_tv = &tv;
	}

	while (length_to_read != 0) {
		rc = rtu_select(ctx, &rset, p_tv, length_to_read);
		if (rc == -1) {
			_error_print(ctx, "select");
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				wsa_err = WSAGetLastError();

				if (wsa_err == WSAENETDOWN || wsa_err == WSAENOTSOCK) {
					rtu_close(ctx);
					rtu_connect(ctx);
				}
				int saved_errno = errno;

				if (errno == ETIMEDOUT) {
					_sleep_response_timeout(ctx);
					rtu_flush(ctx);
				}
				else if (errno == EBADF) {
					rtu_close(ctx);
					rtu_connect(ctx);
				}
				errno = saved_errno;
			}
			return -1;
		}
		rc = rtu_recv(ctx, msg + msg_length, length_to_read);
		if (rc == 0) {
			errno = ECONNRESET;
			rc    = -1;
		}

		if (rc == -1) {
			_error_print(ctx, "read");
			return -1;
		}

		if (ctx->debug) {
			for (s32 i = 0; i < rc; i++) {
				printf("<%.2X>", msg[msg_length + i]);
			}
		}

		msg_length += rc;

		//Compute remaining msg
		length_to_read -= rc;

		if (length_to_read == 0) {
			switch (step)
			{
			case _STEP_FUNCTION: {
				length_to_read = compute_meta_length_after_function(msg[_MODBUS_RTU_HEADER_LENGTH], msg_type);

				if (length_to_read != 0) {
					step = _STEP_META;
				}
			}break;
			case _STEP_META: {
				length_to_read = compute_meta_length_after_meta(ctx, msg, msg_type);

				if ((msg_length + length_to_read) > MODBUS_RTU_MAX_ADU_LENGTH) {
					errno = EMBBADDATA;
					_error_print(ctx, "too many data");
					return -1;
				}
				step = _STEP_DATA;
			}break;
			default:
				break;
			}
		}

		if (length_to_read > 0 && (ctx->byte_timeout.tv_sec > 0 || ctx->byte_timeout.tv_usec > 0)){
			tv.tv_usec = ctx->byte_timeout.tv_usec;
			tv.tv_sec  = ctx->byte_timeout.tv_sec;
			p_tv       = &tv;
		}
	}

	if (ctx->debug) {
		printf("\n");
	}

	return rtu_check_integrity(ctx, msg, msg_length);
}

function modbus_t*
modbus_new_rtu(const char* device, u32 baud, char parity, s32 data_bit, s32 stop_bit) {

	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
	wVersionRequested = MAKEWORD(2, 2);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		/* Tell the user that we could not find a usable */
		/* Winsock DLL.                                  */
		printf("WSAStartup failed with error: %d\n", err);
		return 1;
	}

	modbus_t* ctx;

	if (device == NULL || *device == 0) {
		fprintf(stderr, "This device string is empty ! ");
		errno = EINVAL;
		return NULL;
	}

	if (baud == 0) {
		fprintf(stderr, "The baud rate value most not be zero");
		errno = EINVAL;
		return NULL;
	}

	ctx = (modbus_t*)malloc(sizeof(modbus_t));
	if (ctx)
	{
		_modbus_init_common(ctx);
		ctx->rtu_handle = (modbus_rtu*)malloc(sizeof(modbus_rtu));
		if (ctx->rtu_handle)
		{
			ctx->rtu_handle->device = (char*)malloc(strlen(device) + 1 * sizeof(char));

			if (ctx->rtu_handle->device) {
				strcpy_s(ctx->rtu_handle->device, strlen(device) + 1, device);

				if (parity == 'N' || parity == 'E' || parity == 'O') {
					ctx->rtu_handle->parity = parity;
					ctx->rtu_handle->data_bit = data_bit;
					ctx->rtu_handle->stop_bit = stop_bit;
					ctx->rtu_handle->confirmation_to_ignore = FALSE;
					ctx->rtu_handle->baud = baud;
					ctx->debug = 1;
					return ctx;
				}
				else {
					modbus_free(ctx);
					errno = EINVAL;
					return NULL;
				}
			}
			else {
				modbus_free(ctx);
				errno = EINVAL;
				return NULL;
			}
		}
		else {
			modbus_free(ctx);
			errno = EINVAL;
			return NULL;
		}
	}
	else {
		errno = EINVAL;
		return NULL;
	}
}



function s32 
rtu_send_message_pre(u8* req, s32 req_length) {
	u16 crc = crc16(req, req_length);
	/* According to the MODBUS specs (p. 14), low bytes comes first in the RTU message*/

	req[req_length++] = crc & 0x00FF;
	req[req_length++] = crc >> 8;
	return req_length;
}

function ssize_t 
rtu_send(modbus_t* ctx, const u8* req, s32 req_length) {
	modbus_rtu* ctx_rtu = ctx->rtu_handle;
	DWORD n_bytes = 0;
	return (WriteFile(ctx_rtu->w_ser.fd, req, req_length, &n_bytes, NULL)) ? (ssize_t)n_bytes: -1;
}



function s32 
send_msg(modbus_t* ctx, u8* msg, s32 msg_length) {

	msg_length = rtu_send_message_pre(msg, msg_length);

	if (ctx->debug) {
		for (s32 i = 0; i < msg_length; i++) {
			printf("[%.2X]", msg[i]);
		}
		printf("\n");
	}

	//Use tcp_send if ctx is tcp
	s32 rc = rtu_send(ctx, msg, msg_length);

	if (rc == -1) {
		_error_print(ctx, NULL);
	}

	if (rc > 0 && rc != msg_length) {
		errno = EMBBADDATA;
		return -1;
	}
	return rc;
}

function s32 
read_registers(modbus_t* ctx, s32 func, s32 addr, s32 nb, u16* dest) {
	u8  req[_MIN_REQ_LENGTH];
	u8  rsp[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		if (ctx->debug) {
			fprintf(stderr, "ERROR too many registers requested (%d > %d) \n", nb, MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	s32 req_length = rtu_build_request_basis(ctx, func, addr, nb, req);

	s32 rc = send_msg(ctx, req, req_length);

	if (rc > 0) {
		u32 offset;
		s32 i;
		rc = rtu_receive_message(ctx, rsp, MSG_CONFIRMATION);

		if (rc == -1) {
			return -1;
		}

		offset = _MODBUS_RTU_HEADER_LENGTH;
		for (i = 0; i < rc; i++) {
			dest[i] = (rsp[offset + 2 + (i << 1)] << 8) | rsp[offset + 3 + (i << 1)];
		}
	}
	return rc;
}


function int 
modbus_read_registers(modbus_t* ctx, u32 addr, u32 nb, u16* dest) {
	s32 status;

	if (ctx != NULL) {

		if (nb <= MODBUS_MAX_READ_REGISTERS) {
			status = read_registers(ctx, MODBUS_FC_READ_HOLDING_REGISTERS, addr, nb, dest);
			return status;
		}
		else {
			if (ctx->debug) {
				fprintf(stderr, "Error to many regisers requested (%d > %d) \n", nb, MODBUS_MAX_READ_REGISTERS);
			}
			errno = EMBMDATA;
			return -1;
		}
	}
	else {
		errno = EINVAL;
		return -1;
	}
}

function int
rtu_set_slave(modbus_t* ctx, s32 slave) {
	s32 max_slave = (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;

	if (slave >= 0 && slave <= max_slave) {
		ctx->slave = slave;
	}
	else {
		errno = EINVAL;
		return -1;
	}
	return 0;
}

function void 
test_modbus() {
	modbus_t* ctx = modbus_new_rtu("COM7", 9600, 'E', 8, 1);
	assert(ctx != NULL);
	assert(rtu_set_slave(ctx, 1) == 0);
	assert(rtu_connect(ctx)      == 0);
	u16 dest[12];
	assert(modbus_read_registers(ctx, 0, 2, dest) >= 0);
	printf("%d\n", (u16) dest[0]);
}

int main() {
	test_modbus();
	return 0;
}




