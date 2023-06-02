#include <time.h>
#include <errno.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "modbus_constants.h"


#define MSG_LENGTH_UNDEFINED -1

typedef struct timeval timeval;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint16_t u16;
typedef int32_t  s32;
typedef int64_t  s64;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint8_t  u8;
typedef s8       b8;

#ifdef WIN32
typedef u32      ssize_t;
#endif

#define function               static
#define global_variable        static
#define local_persist          static


typedef struct _sft {
    u32 slave;
    u32 func;
    u32 t_id;
}sft_t;


#define PY_BUF_SIZE 512

typedef struct {
    /* device : "/dev/ttyS0", "/dev/ttyUSB0" etc */
    /* baud : 9600, 19200, 57600, 115200, etc */
    /* DCB : win32, defines the settings for serial communication device*/

    char* device;
    s32       baud;
    u8        data_bit;
    u8        stop_bit;
    char      parity;
#if WIN32
    win32_ser w_ser;
    DCB old_dcb;
#endif

    //To handle many slaves on the same link
    s32       confirmation_to_ignore;

    struct termios old_tios;
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
    timeval response_timeout;
    timeval byte_timeout;
    timeval indication_timeout;

    //if this is a rtu
    //TODO: add modbus_tcp
    modbus_rtu* rtu_handle;
}modbus_t;


function void
modbus_free(modbus_t* ctx){
    if(ctx == NULL){
        return;
    }
    free(ctx->rtu_handle);
    free(ctx);
}

function void
modbus_init_common(modbus_t *ctx){
    ctx->slave                       = -1;
    ctx->s                           = -1;
    ctx->debug                       = false;
    ctx->quirks                      = MODBUS_QUIRK_NONE;
    ctx->error_recovery              = MODBUS_ERROR_RECOVERY_NONE;
    ctx->response_timeout.tv_sec     = 0;
    ctx->response_timeout.tv_usec    = _RESPONSE_TIMEOUT;
    ctx->byte_timeout.tv_sec         = 0;
    ctx->byte_timeout.tv_usec        = _BYTE_TIMEOUT;
    ctx->indication_timeout.tv_sec   = 0;
    ctx->indication_timeout.tv_usec  = 0;
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


void _error_print(modbus_t *ctx, const char *context)
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

function modbus_t*
modbus_new_rtu(const char* device , u32 baud, char parity, s32 data_bit, s32 stop_bit){
    modbus_t*     ctx;
    modbus_rtu*   ctx_rtu;

    assert(device != NULL);
    assert(*device != 0);
    assert(baud > 0);

    ctx = (modbus_t*) malloc(sizeof(modbus_t));

    assert(ctx != NULL);

    modbus_init_common(ctx);
    ctx->rtu_handle = (modbus_rtu*) malloc(sizeof(modbus_rtu));
    if(ctx->rtu_handle == NULL){
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }

    ctx_rtu = (modbus_rtu*) ctx->rtu_handle;
    ctx_rtu->device = (char*)malloc(strlen(device) + 1 * sizeof(char));
    if(ctx_rtu->device == NULL){
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }

    strcpy(ctx_rtu->device, device);

    ctx_rtu->baud = baud;
    if(parity == 'N' || parity == 'E' || parity == 'O'){
        ctx_rtu->parity = parity;
    }else{
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }
    ctx_rtu->data_bit = data_bit;
    ctx_rtu->stop_bit = stop_bit;
    ctx_rtu->confirmation_to_ignore = false;
    return ctx;
}

function speed_t get_termios_speed(int baud, int debug)
{
    speed_t speed;

    switch (baud) {
        case 110:
            speed = B110;
            break;
        case 300:
            speed = B300;
            break;
        case 600:
            speed = B600;
            break;
        case 1200:
            speed = B1200;
            break;
        case 2400:
            speed = B2400;
            break;
        case 4800:
            speed = B4800;
            break;
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
#ifdef B57600
        case 57600:
            speed = B57600;
            break;
#endif
#ifdef B115200
        case 115200:
            speed = B115200;
            break;
#endif
#ifdef B230400
        case 230400:
            speed = B230400;
            break;
#endif
#ifdef B460800
        case 460800:
            speed = B460800;
            break;
#endif
#ifdef B500000
        case 500000:
            speed = B500000;
            break;
#endif
#ifdef B576000
        case 576000:
            speed = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            speed = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            speed = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            speed = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            speed = B1500000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            speed = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            speed = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            speed = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            speed = B4000000;
            break;
#endif
        default:
            speed = B9600;
            if (debug) {
                fprintf(stderr, "WARNING Unknown baud rate %d (B9600 used)\n", baud);
            }
    }

    return speed;
}

/* Computes the length of the expected response */
function unsigned int compute_response_length_from_request(modbus_t *ctx, uint8_t *req)
{
    int length;
    const int offset = _MODBUS_RTU_HEADER_LENGTH;

    switch (req[offset]) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        /* Header + nb values (code from write_bits) */
        int nb = (req[offset + 3] << 8) | req[offset + 4];
        length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
    } break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        /* Header + 2 * nb values */
        length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case MODBUS_FC_REPORT_SLAVE_ID:
        /* The response is device specific (the header provides the
           length) */
        return MSG_LENGTH_UNDEFINED;
    case MODBUS_FC_MASK_WRITE_REGISTER:
        length = 7;
        break;
    default:
        length = 5;
    }

    return offset + length + _MODBUS_RTU_CHECKSUM_LENGTH;
}

function s32 rtu_pre_check_confirmation(modbus_t *ctx,
                                              const uint8_t *req,
                                              const uint8_t *rsp,
                                              int rsp_length)
{
    /* Check responding slave is the slave we requested (except for broacast
     * request) */
    if (req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "The responding slave %d isn't the requested slave %d\n",
                    rsp[0],
                    req[0]);
        }
        errno = EMBBADSLAVE;
        return -1;
    } else {
        return 0;
    }
}

function void 
_sleep_response_timeout(modbus_t *ctx)
{
    struct timespec request, remaining;
    request.tv_sec = ctx->response_timeout.tv_sec;
    request.tv_nsec = ((long int) ctx->response_timeout.tv_usec) * 1000;
    while (nanosleep(&request, &remaining) == -1 && errno == EINTR) {
        request = remaining;
    }
}


function s32
check_confirmation(modbus_t *ctx, uint8_t *req, uint8_t *rsp, int rsp_length)
{
    s32 rc;
    s32 rsp_length_computed;
    const unsigned int offset = _MODBUS_RTU_HEADER_LENGTH;
    const int func= rsp[offset];

    rc = rtu_pre_check_confirmation(ctx, req, rsp, rsp_length);
    if (rc == -1) {
      if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
        _sleep_response_timeout(ctx);
        tcflush(ctx->s, TCIOFLUSH);
      }
      return -1;
    }

    rsp_length_computed = compute_response_length_from_request(ctx, req);

    /* Exception code */
    if (func>= 0x80) {
        if (rsp_length == (int) (offset + 2 + _MODBUS_RTU_CHECKSUM_LENGTH) &&
            req[offset] == (rsp[offset] - 0x80)) {
            /* Valid exception code received */

            int exception_code = rsp[offset + 1];
            if (exception_code < MODBUS_EXCEPTION_MAX) {
                errno = MODBUS_ENOBASE + exception_code;
            } else {
                errno = EMBBADEXC;
            }
            _error_print(ctx, NULL);
            return -1;
        } else {
            errno = EMBBADEXC;
            _error_print(ctx, NULL);
            return -1;
        }
    }

    /* Check length */
    if ((rsp_length == rsp_length_computed ||
         rsp_length_computed == MSG_LENGTH_UNDEFINED) &&
    func< 0x80) {
        int req_nb_value;
        int rsp_nb_value;
        int resp_addr_ok = true;
        int resp_data_ok = true;

        /* Check function code */
        if (func != req[offset]) {
            if (ctx->debug) {
                fprintf(
                        stderr,
                        "Received function not corresponding to the request (0x%X != 0x%X)\n",
                func ,
                req[offset]);
            }
            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                    tcflush(ctx->s, TCIOFLUSH);
            }
            errno = EMBBADDATA;
            return -1;
        }

        /* Check the number of values is corresponding to the request */
        switch (func ) {
            case MODBUS_FC_READ_COILS:
            case MODBUS_FC_READ_DISCRETE_INPUTS:
                /* Read functions, 8 values in a byte (nb
                 * of values in the request and byte count in
                 * the response. */
                req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
            rsp_nb_value = rsp[offset + 1];
            break;
            case MODBUS_FC_WRITE_AND_READ_REGISTERS:
            case MODBUS_FC_READ_HOLDING_REGISTERS:
            case MODBUS_FC_READ_INPUT_REGISTERS:
                /* Read functions 1 value = 2 bytes */
                req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 1] / 2);
            break;
            case MODBUS_FC_WRITE_MULTIPLE_COILS:
            case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                /* address in request and response must be equal */
                if ((req[offset + 1] != rsp[offset + 1]) ||
                    (req[offset + 2] != rsp[offset + 2])) {
                    resp_addr_ok = false;
                }
            /* N Write functions */
            req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
            rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
            break;
            case MODBUS_FC_REPORT_SLAVE_ID:
                /* Report slave ID (bytes received) */
                req_nb_value = rsp_nb_value = rsp[offset + 1];
            break;
            case MODBUS_FC_WRITE_SINGLE_COIL:
            case MODBUS_FC_WRITE_SINGLE_REGISTER:
                /* address in request and response must be equal */
                if ((req[offset + 1] != rsp[offset + 1]) ||
                    (req[offset + 2] != rsp[offset + 2])) {
                    resp_addr_ok = false;
                }
            /* data in request and response must be equal */
            if ((req[offset + 3] != rsp[offset + 3]) ||
                (req[offset + 4] != rsp[offset + 4])) {
                resp_data_ok = false;
            }
            /* 1 Write functions & others */
            req_nb_value = rsp_nb_value = 1;
            break;
            default:
                /* 1 Write functions & others */
                req_nb_value = rsp_nb_value = 1;
            break;
        }

        if ((req_nb_value == rsp_nb_value) && (resp_addr_ok == true) &&
            (resp_data_ok == true)) {
            rc = rsp_nb_value;
        } else {
            if (ctx->debug) {
                fprintf(stderr,
                        "Received data not corresponding to the request (%d != %d)\n",
                        rsp_nb_value,
                        req_nb_value);
            }

            if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
                _sleep_response_timeout(ctx);
                    tcflush(ctx->s, TCIOFLUSH);
            }

            errno = EMBBADDATA;
            rc = -1;
        }
    } else {
        if (ctx->debug) {
            fprintf(
                    stderr,
                    "Message length not corresponding to the computed length (%d != %d)\n",
                    rsp_length,
                    rsp_length_computed);
        }
        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _sleep_response_timeout(ctx);
                    tcflush(ctx->s, TCIOFLUSH);
        }
        errno = EMBBADDATA;
        rc = -1;
    }

    return rc;
}





function s32
modbus_connect(modbus_t* ctx){
    struct termios tios;
    speed_t speed;
    modbus_rtu *ctx_rtu = ctx->rtu_handle;

    if(ctx->debug){
        printf("Opening %s at %d bauds (%c, %d, %d)\n",
               ctx_rtu->device,
               ctx_rtu->baud,
               ctx_rtu->parity,
               ctx_rtu->data_bit,
               ctx_rtu->stop_bit
               );
    }

    s32 flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
    ctx->s = open(ctx_rtu->device, flags);
    tcgetattr(ctx->s, &ctx_rtu->old_tios);
    memset(&tios, 0, sizeof(struct termios));

    speed = get_termios_speed(ctx_rtu->baud, ctx->debug);

    if((cfsetispeed(&tios, speed) < 0) || (cfsetospeed(&tios, speed))){
        close(ctx->s);
        ctx->s = -1;
        return -1;
    }

    tios.c_cflag |= ( CREAD | CLOCAL);

    tios.c_cflag &= ~ CSIZE;

    switch (ctx_rtu->data_bit) {
        case 5:{
            tios.c_cflag |= CS5;
        } break;
        case 6:{
            tios.c_cflag |= CS6;
        }break;
        case 7:{
            tios.c_cflag |= CS7;
        }break;
        default:{
            tios.c_cflag |= CS8;
        }break;

    }
    if(ctx_rtu->stop_bit == 1){
        tios.c_cflag &= ~CSTOPB;
    }else{
        tios.c_cflag != CSTOPB;
    }

    if (ctx_rtu->parity == 'N') {
        tios.c_cflag &= ~PARENB;
    } else if (ctx_rtu->parity == 'E') {
        tios.c_cflag |= PARENB;
        tios.c_cflag &= ~PARODD;
    } else {
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
    }
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    if (ctx_rtu->parity == 'N') {
        /* None */
        tios.c_iflag &= ~INPCK;
    } else {
        tios.c_iflag |= INPCK;
    }
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);
    tios.c_oflag &= ~OPOST;
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    if (tcsetattr(ctx->s, TCSANOW, &tios) < 0) {
        close(ctx->s);
        ctx->s = -1;
        return -1;
    }

    return 0;
}

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

function s32
rtu_build_request_basis(modbus_t* ctx, s32 func, s32 addr, s32 nb, u8 * req){
    assert(ctx->slave != -1);
    req[0] = ctx->slave;
    req[1] = func;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;

    return _MODBUS_RTU_PRESET_REQ_LENGTH;
}

function void
modbus_rtu_close(modbus_t* ctx){
    modbus_rtu* ctx_rtu = ctx->rtu_handle;

    if(ctx->s >= 0){
        tcsetattr(ctx->s, TCSANOW, &ctx_rtu->old_tios);
        close(ctx->s);
        ctx->s = -1;
    }
}



function s32
send_msg(modbus_t* ctx, u8* msg, s32 msg_length){

    u16 crc = crc16(msg, msg_length);
    msg[msg_length++] = crc & 0x00ff;
    msg[msg_length++] = crc >> 8;

    if(ctx->debug){
        for (int  i = 0; i < msg_length; i++){
            printf("[%.2X],", msg[i]);
        }
        printf("\n");
    }

    int rc;
    do{
        rc = write(ctx->s, msg, msg_length);

        if(rc == -1){
            _error_print(ctx, NULL);

            if(ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK){
                s32 saved_errno = errno;

                if((errno == EBADF || errno == ECONNRESET || errno == EPIPE)){
                    modbus_connect(ctx);
                    _sleep_response_timeout(ctx);
                    modbus_connect(ctx);
                }else{
                    _sleep_response_timeout(ctx);
                    tcflush(ctx->s, TCIOFLUSH);
                }
                errno = saved_errno;
            }
        }
    }while((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) && rc == -1);

    if(rc > 0 && rc != msg_length){
        errno = EMBBADDATA;
        return -1;
    }

    return rc;
}

function s32
rtu_select(modbus_t* ctx, fd_set* rset, timeval* tv, s32 length_to_read){
    s32 s_rc;
    while((s_rc = select(ctx->s + 1, rset, NULL, NULL , tv)) == -1){
        if(errno == EINTR){
            FD_ZERO(rset);
            FD_SET(ctx->s, rset);
        }else{
            return -1;
        }
    }

    if(s_rc == 0){
        errno = ETIMEDOUT;
        return -1;
    }
    return s_rc;
}
/*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */

/* Computes the length to read after the function received */
function u8 compute_meta_length_after_function(int func, msg_type_t msg_type)
{
    int length;

    if (msg_type == MSG_INDICATION) {
        if (func<= MODBUS_FC_WRITE_SINGLE_REGISTER) {
            length = 4;
        } else if (func== MODBUS_FC_WRITE_MULTIPLE_COILS ||
        func== MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
            length = 5;
        } else if (func== MODBUS_FC_MASK_WRITE_REGISTER) {
            length = 6;
        } else if (func== MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = 9;
        } else {
            /* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
            length = 0;
        }
    } else {
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

/* Computes the length to read after the meta information (address, count, etc) */
function int
compute_data_length_after_meta(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type)
{
    int func= _MODBUS_RTU_HEADER_LENGTH;
    int length;

    if (msg_type == MSG_INDICATION) {
        switch (func) {
            case MODBUS_FC_WRITE_MULTIPLE_COILS:
            case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                length = msg[_MODBUS_RTU_HEADER_LENGTH + 5];
            break;
            case MODBUS_FC_WRITE_AND_READ_REGISTERS:
                length = msg[_MODBUS_RTU_HEADER_LENGTH + 9];
            break;
            default:
                length = 0;
        }
    } else {
        /* MSG_CONFIRMATION */
        if (func<= MODBUS_FC_READ_INPUT_REGISTERS ||
        func== MODBUS_FC_REPORT_SLAVE_ID ||
        func== MODBUS_FC_WRITE_AND_READ_REGISTERS) {
            length = msg[_MODBUS_RTU_HEADER_LENGTH + 1];
        } else {
            length = 0;
        }
    }

    length += _MODBUS_RTU_CHECKSUM_LENGTH;

    return length;
}

function s32
rtu_check_integrity(modbus_t* ctx, u8* msg, s32 msg_length){
    s32 slave = msg[0];

    if(slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS){
        if(ctx->debug){
            printf("Request for slave %d ignored (not %d)\n", slave, ctx->slave);
        }
        return 0;
    }

    u16 crc_calculated = crc16(msg, msg_length - 2);
    u16 crc_received   = (msg[msg_length -1] << 8) || msg[msg_length - 2];

    if(crc_calculated == crc_received){
        return msg_length;
    }
    else{
        if(ctx->debug){
            fprintf(stderr, "ERROR CRC received 0x%0X != CRC calculated 0x%0X\n", crc_received, crc_calculated);
        }

        if(ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL){
            tcflush(ctx->s, TCIOFLUSH);
        }
        errno = EMBBADCRC;
        return -1;
    }

}

function void
modbus_close(modbus_t* ctx){
    modbus_rtu* ctx_rtu = ctx->rtu_handle;

    if(ctx->s >= 0){
        tcsetattr(ctx->s, TCSANOW, &ctx_rtu->old_tios);
        close(ctx->s);
        ctx->s = -1;
    }
}

function s32
modbus_receive_msg(modbus_t* ctx, u8* msg, msg_type_t msg_type){
    int rc;
    fd_set rset;
    timeval tv;
    timeval *p_tv;

    u32 msg_length = 0;

    _step_t step;

    if(ctx->debug){
        if(msg_type == MSG_INDICATION){
            printf("Waiting for an indcation... \n");
        }else{
            printf("Waiting for an confirmation... \n");
        }
    }

    if(ctx->s <= 0){
        if(ctx->debug){
            fprintf(stderr, "Error the connection is not establish.\n");
        }
        return -1;
    }

    FD_ZERO(&rset);
    FD_SET(ctx->s, &rset);

    step = _STEP_FUNCTION;
    u32 len_to_read = _MODBUS_RTU_HEADER_LENGTH + 1;
    if(msg_type == MSG_INDICATION){
        if(ctx->indication_timeout.tv_sec == 0 && ctx->indication_timeout.tv_usec == 0){
            p_tv = NULL;
        }else{
            tv.tv_sec = ctx->indication_timeout.tv_sec;
            tv.tv_usec = ctx->indication_timeout.tv_usec;
            p_tv = &tv;
        }
    }else{
        tv.tv_sec = ctx->response_timeout.tv_sec;
        tv.tv_usec = ctx->response_timeout.tv_usec;
        p_tv = &tv;
    }

    while(len_to_read != 0){
        rc = rtu_select(ctx, &rset, p_tv, len_to_read);

        if(rc == -1){
            _error_print(ctx, "select");
            if(ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK){
                s32 saved_errno = errno;
                if (errno == ETIMEDOUT) {
                    _sleep_response_timeout(ctx);
                    tcflush(ctx->s, TCIOFLUSH);
                } else if (errno == EBADF) {
                    tcflush(ctx->s, TCIOFLUSH);
                    modbus_connect(ctx);
                }
                errno = saved_errno;
            }

            return -1;
        }
        rc = read(ctx->s, msg + msg_length, len_to_read);

        if(rc == 0){
            errno = ECONNRESET;
            rc = -1;
        }

        if(rc == -1){
            _error_print(ctx, "read");

            if ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) &&
                (errno == ECONNRESET || errno == ECONNREFUSED || errno == EBADF)) {
                int saved_errno = errno;
                modbus_close(ctx);
                modbus_connect(ctx);
                /* Could be removed by previous calls */
                errno = saved_errno;
            }
            return -1;
        }

        if(ctx->debug){
            for (int i = 0; i < rc; ++i) {
                printf("<%.2X>", msg[msg_length + i]);
            }
        }

        msg_length += rc;
        len_to_read -= rc;

        if(len_to_read == 0){
            switch (step) {
                case _STEP_FUNCTION:{
                    len_to_read = compute_meta_length_after_function(msg[_MODBUS_RTU_HEADER_LENGTH], msg_type);

                    if(len_to_read != 0){
                        step = _STEP_META;
                        break;
                    }
                }break;
                case _STEP_META:{
                    len_to_read = compute_data_length_after_meta(ctx, msg, msg_type);

                    if((msg_length + len_to_read) > MODBUS_RTU_MAX_ADU_LENGTH){
                        errno = EMBBADDATA;
                        _error_print(ctx, "too many data");
                        return -1;
                    }
                    step = _STEP_DATA;
                }break;
                default:break;
            }
        }
        if (len_to_read > 0 &&
            (ctx->byte_timeout.tv_sec > 0 || ctx->byte_timeout.tv_usec > 0)) {
            /* If there is no character in the buffer, the allowed timeout
               interval between two consecutive bytes is defined by
               byte_timeout */
            tv.tv_sec = ctx->byte_timeout.tv_sec;
            tv.tv_usec = ctx->byte_timeout.tv_usec;
            p_tv = &tv;
        }
    }

    if(ctx->debug){
        printf("\n");
    }

    return rtu_check_integrity(ctx, msg, msg_length);
}

function s32
read_registers(modbus_t* ctx, s32 func, s32 addr, s32 nb, u16 * dest){
    s32 rc;
    s32 req_length;
    u8 req[_MIN_REQ_LENGTH];
    u8 rsp[MAX_MESSAGE_LENGTH];

    if(nb > MODBUS_MAX_READ_REGISTERS){
        if(ctx->debug){
            fprintf(stderr, "ERROR too many registers requested (%d > %d)\n", nb, MODBUS_MAX_READ_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    req_length = rtu_build_request_basis(ctx, func, addr, nb, req);

    rc = send_msg(ctx, req, req_length);

    if(rc >0){
        u32 offset;
        s32 i;

        rc = modbus_receive_msg(ctx, rsp, MSG_CONFIRMATION);
        if(rc == -1) return -1;

        rc = check_confirmation(ctx, req, rsp, rc);

        if(rc == -1)return -1;

        offset = _MODBUS_RTU_HEADER_LENGTH;

        for(int i = 0; i < rc; i++){
            dest[i] = (rsp[offset + 2 + (i << 1)] << 8) | rsp[offset + 3 + (i << 1)];
        }
    }

    return rc;
}

function s32
rtu_set_slave(modbus_t* ctx, s32 slave){
    s32 max_slave =  (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;

    if(slave >= 0 && slave <= max_slave){
        ctx->slave = slave;
    }
    else{
        errno = EINVAL;
        return -1;
    }
    return 0;
}

function s32 modbus_set_slave(modbus_t* ctx, s32 slave){
    if(ctx == NULL){
        errno = EINVAL;
        return -1;
    }

    s32 max_slave = (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;
    if(slave >= 0 && slave <= max_slave){
        ctx->slave = slave;
    }else{
        errno = EINVAL;
        return -1;
    }
    return 0;
}

function s32
modbus_read_registers(modbus_t* ctx, s32 addr, s32 nb, u16* dest){
    s32 status;
    if(ctx == NULL){
        errno = EINVAL;
        return -1;
    }

    if(nb > MODBUS_MAX_READ_REGISTERS){
        if(ctx->debug){
            fprintf(stderr, "Err to many register requested (%d > %d)\n", nb, MODBUS_MAX_READ_REGISTERS);
        }
        errno = EMBMDATA;
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_HOLDING_REGISTERS, addr, nb, dest);
    return status;
}

function void
test_modbus(){
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'E', 8, 1);
    modbus_set_slave(ctx, 1);
    s32 ret = modbus_connect(ctx);
    printf("ret = %d\n", ret);
    u16 data[2];

    ret = modbus_read_registers(ctx, 0, 1 , data );

    printf("ret = %d\n", ret);
    printf("%d\n", data[0]);
    printf("%d\n", data[1]);
    modbus_close(ctx);
}



int main(){
    test_modbus();
    return 0;
}
