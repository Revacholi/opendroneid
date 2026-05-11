/*
 * mavlink_reader.c
 *
 * Reads MAVLink v2 frames from a serial (USB) port connected to Pixhawk.
 * Decodes OPEN_DRONE_ID_* messages and GLOBAL_POSITION_INT, and pushes
 * them into a thread-safe bounded queue consumed by the ODID state machine.
 *
 * Uses the MAVLink C library headers directly (no external library needed).
 */

#include "mavlink_reader.h"
#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* MAVLink C library — ardupilotmega dialect includes all ODID messages */
#include <ardupilotmega/mavlink.h>


#define QUEUE_CAP 64

typedef struct {
    odid_queue_msg_t buf[QUEUE_CAP];
    int head, tail, count;
    pthread_mutex_t  mu;
    pthread_cond_t   not_empty;
    pthread_cond_t   not_full;
} msg_queue_t;

static msg_queue_t s_queue;

static void queue_init(msg_queue_t *q) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
}

static void queue_push(msg_queue_t *q, const odid_queue_msg_t *msg) {
    pthread_mutex_lock(&q->mu);
    while (q->count == QUEUE_CAP)
        pthread_cond_wait(&q->not_full, &q->mu);
    q->buf[q->tail] = *msg;
    q->tail = (q->tail + 1) % QUEUE_CAP;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mu);
}

/* Returns 1 on success, 0 on timeout */
static int queue_pop(msg_queue_t *q, odid_queue_msg_t *out, int timeout_ms) {
    pthread_mutex_lock(&q->mu);
    if (q->count == 0 && timeout_ms != 0) {
        if (timeout_ms < 0) {
            pthread_cond_wait(&q->not_empty, &q->mu);
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec  += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
            }
            pthread_cond_timedwait(&q->not_empty, &q->mu, &ts);
        }
    }
    if (q->count == 0) {
        pthread_mutex_unlock(&q->mu);
        return 0;
    }
    *out = q->buf[q->head];
    q->head = (q->head + 1) % QUEUE_CAP;
    q->count--;
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mu);
    return 1;
}


static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            LOG_WARN("mavlink: unknown baud %d, using 921600", baud);
            return B921600;
    }
}

static int serial_open(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        LOG_ERROR("mavlink: open %s: %s", dev, strerror(errno));
        return -1;
    }
    fcntl(fd, F_SETFL, 0);  /* blocking */

    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) {
        LOG_ERROR("mavlink: tcgetattr: %s", strerror(errno));
        close(fd);
        return -1;
    }
    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;  /* 0.1 s inter-char timeout */

    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        LOG_ERROR("mavlink: tcsetattr: %s", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}


static int udp_open(const char *host, uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        LOG_ERROR("mavlink: UDP socket: %s", strerror(errno));
        return -1;
    }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        LOG_ERROR("mavlink: invalid UDP host '%s'", host);
        close(fd);
        return -1;
    }
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("mavlink: UDP bind %s:%u: %s", host, port, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}


static void dispatch_mavlink(const mavlink_message_t *msg) {
    odid_queue_msg_t qmsg;
    memset(&qmsg, 0, sizeof(qmsg));

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t m;
        mavlink_msg_open_drone_id_basic_id_decode(msg, &m);
        qmsg.type = ODID_MSG_BASIC_ID;
        memcpy(qmsg.raw, &m, sizeof(m));
        qmsg.raw_len = sizeof(m);
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: BASIC_ID id_type=%d ua_type=%d", m.id_type, m.ua_type);
        break;
    }

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: {
        mavlink_open_drone_id_location_t m;
        mavlink_msg_open_drone_id_location_decode(msg, &m);
        qmsg.type = ODID_MSG_LOCATION;
        memcpy(qmsg.raw, &m, sizeof(m));
        qmsg.raw_len = sizeof(m);
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: LOCATION lat=%d lon=%d alt=%.1f",
                  m.latitude, m.longitude, (double)m.altitude_geodetic);
        break;
    }

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: {
        mavlink_open_drone_id_system_t m;
        mavlink_msg_open_drone_id_system_decode(msg, &m);
        qmsg.type = ODID_MSG_SYSTEM;
        memcpy(qmsg.raw, &m, sizeof(m));
        qmsg.raw_len = sizeof(m);
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: SYSTEM op_lat=%d op_lon=%d", m.operator_latitude, m.operator_longitude);
        break;
    }

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: {
        mavlink_open_drone_id_operator_id_t m;
        mavlink_msg_open_drone_id_operator_id_decode(msg, &m);
        qmsg.type = ODID_MSG_OPERATOR_ID;
        memcpy(qmsg.raw, &m, sizeof(m));
        qmsg.raw_len = sizeof(m);
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: OPERATOR_ID type=%d", m.operator_id_type);
        break;
    }

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: {
        mavlink_open_drone_id_self_id_t m;
        mavlink_msg_open_drone_id_self_id_decode(msg, &m);
        qmsg.type = ODID_MSG_SELF_ID;
        memcpy(qmsg.raw, &m, sizeof(m));
        qmsg.raw_len = sizeof(m);
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: SELF_ID type=%d", m.description_type);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t m;
        mavlink_msg_global_position_int_decode(msg, &m);
        qmsg.type       = ODID_MSG_GPS_INT;
        qmsg.gps.lat    = m.lat;
        qmsg.gps.lon    = m.lon;
        qmsg.gps.alt    = m.alt;
        qmsg.gps.relative_alt = m.relative_alt;
        qmsg.gps.vx     = m.vx;
        qmsg.gps.vy     = m.vy;
        qmsg.gps.vz     = m.vz;
        qmsg.gps.hdg    = m.hdg;
        queue_push(&s_queue, &qmsg);
        LOG_DEBUG("mavlink: GPS lat=%d lon=%d alt=%d hdg=%d",
                  m.lat, m.lon, m.alt, m.hdg);
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t m;
        mavlink_msg_heartbeat_decode(msg, &m);
        /* Only care about the autopilot heartbeat (component 1) */
        if (msg->compid == MAV_COMP_ID_AUTOPILOT1) {
            qmsg.type         = ODID_MSG_HEARTBEAT;
            qmsg.hb.base_mode = m.base_mode;
            qmsg.hb.system_status = m.system_status;
            queue_push(&s_queue, &qmsg);
            LOG_DEBUG("mavlink: HEARTBEAT base_mode=0x%02x status=%d",
                      m.base_mode, m.system_status);
        }
        break;
    }

    default:
        break;
    }
}


static struct {
    char     device[64];
    int      baud;
    bool     is_udp;
    char     udp_host[64];
    uint16_t udp_port;
    int      fd;
    bool     running;
    pthread_t tid;
    /* UDP: destination address learned from first received packet */
    struct sockaddr_in remote_addr;
    socklen_t          remote_addr_len;
    bool               remote_known;
} s_reader;


static void send_heartbeat(void) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_ODID_TXRX_1, &msg,
                               MAV_TYPE_ODID, MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_ACTIVE);
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    if (s_reader.is_udp) {
        if (s_reader.remote_known)
            sendto(s_reader.fd, buf, len, 0,
                   (struct sockaddr *)&s_reader.remote_addr,
                   s_reader.remote_addr_len);
    } else if (s_reader.fd >= 0) {
        (void)write(s_reader.fd, buf, len);
    }
}

static void *reader_thread(void *arg) {
    (void)arg;
    mavlink_message_t msg;
    mavlink_status_t  status;

    if (s_reader.is_udp) {
        s_reader.fd = udp_open(s_reader.udp_host, s_reader.udp_port);
        if (s_reader.fd < 0) {
            LOG_ERROR("mavlink: UDP init failed — reader thread exiting");
            return NULL;
        }
        LOG_INFO("mavlink: listening on UDP %s:%u", s_reader.udp_host, s_reader.udp_port);

        uint8_t buf[512];
        long long last_hb = 0;
        while (s_reader.running) {
            struct pollfd pfd = { .fd = s_reader.fd, .events = POLLIN };
            int ret = poll(&pfd, 1, 200);
            if (ret < 0) {
                if (errno == EINTR) continue;
                LOG_ERROR("mavlink: UDP poll error: %s", strerror(errno));
                break;
            }

            if (ret > 0) {
                struct sockaddr_in from;
                socklen_t from_len = sizeof(from);
                ssize_t n = recvfrom(s_reader.fd, buf, sizeof(buf), 0,
                                     (struct sockaddr *)&from, &from_len);
                if (n < 0) {
                    if (errno == EINTR) continue;
                    LOG_ERROR("mavlink: UDP recvfrom error: %s", strerror(errno));
                    break;
                }
                if (!s_reader.remote_known) {
                    s_reader.remote_addr     = from;
                    s_reader.remote_addr_len = from_len;
                    s_reader.remote_known    = true;
                }
                for (ssize_t i = 0; i < n; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                        dispatch_mavlink(&msg);
                }
            }

            long long now = time_ms();
            if (now - last_hb >= 1000) {
                send_heartbeat();
                last_hb = now;
            }
        }
        LOG_INFO("mavlink: reader thread stopped");
        return NULL;
    }

    uint8_t byte;
    long long last_hb = 0;
    LOG_INFO("mavlink: reader thread started, waiting for %s", s_reader.device);

    /* Open the port — retry until it appears (handles boot-time race and
     * the case where the FC is plugged in after the daemon starts). */
    while (s_reader.running && s_reader.fd < 0) {
        s_reader.fd = serial_open(s_reader.device, s_reader.baud);
        if (s_reader.fd >= 0) {
            LOG_INFO("mavlink: connected to %s @ %d", s_reader.device, s_reader.baud);
            break;
        }
        LOG_WARN("mavlink: %s not available, retrying in 2s...", s_reader.device);
        usleep(2000000);
    }

    while (s_reader.running) {
        /* Use poll() so we detect POLLHUP/POLLERR when the USB device is
         * unplugged — a blocking read() would hang indefinitely on Linux CDC. */
        struct pollfd pfd = { .fd = s_reader.fd, .events = POLLIN };
        int ret = poll(&pfd, 1, 1000);  /* 1 s timeout keeps the loop responsive */

        if (ret < 0) {
            if (errno == EINTR) continue;
            LOG_ERROR("mavlink: poll error: %s — reconnecting", strerror(errno));
            goto reconnect;
        }
        if (ret == 0) {
            long long now = time_ms();
            if (now - last_hb >= 1000) {
                send_heartbeat();
                last_hb = now;
            }
            continue;
        }

        if (pfd.revents & (POLLHUP | POLLERR)) {
            LOG_ERROR("mavlink: device disconnected — reconnecting");
            goto reconnect;
        }

        ssize_t n = read(s_reader.fd, &byte, 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            LOG_ERROR("mavlink: read error: %s — reconnecting", strerror(errno));
            goto reconnect;
        }
        if (n == 0) continue;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
            dispatch_mavlink(&msg);

        long long now = time_ms();
        if (now - last_hb >= 1000) {
            send_heartbeat();
            last_hb = now;
        }
        continue;

    reconnect:
        close(s_reader.fd);
        s_reader.fd = -1;
        mavlink_reset_channel_status(MAVLINK_COMM_0);
        while (s_reader.running) {
            usleep(2000000);  /* 2 s between attempts */
            s_reader.fd = serial_open(s_reader.device, s_reader.baud);
            if (s_reader.fd >= 0) {
                LOG_INFO("mavlink: reconnected to %s", s_reader.device);
                break;
            }
            LOG_WARN("mavlink: %s not available, retrying...", s_reader.device);
        }
    }

    LOG_INFO("mavlink: reader thread stopped");
    return NULL;
}


int mavlink_reader_init(const char *device, int baud) {
    memset(&s_reader, 0, sizeof(s_reader));
    strncpy(s_reader.device, device, sizeof(s_reader.device) - 1);
    s_reader.baud    = baud;
    s_reader.is_udp  = false;
    s_reader.fd      = -1;
    queue_init(&s_queue);
    return 0;
}

int mavlink_reader_init_udp(const char *host, uint16_t port) {
    memset(&s_reader, 0, sizeof(s_reader));
    strncpy(s_reader.udp_host, host, sizeof(s_reader.udp_host) - 1);
    s_reader.udp_port = port;
    s_reader.is_udp   = true;
    s_reader.fd       = -1;
    queue_init(&s_queue);
    return 0;
}

int mavlink_reader_start(void) {
    s_reader.running = true;
    if (pthread_create(&s_reader.tid, NULL, reader_thread, NULL) != 0) {
        LOG_ERROR("mavlink: pthread_create: %s", strerror(errno));
        s_reader.running = false;
        return -1;
    }
    return 0;
}

int mavlink_reader_pop(odid_queue_msg_t *out, int timeout_ms) {
    return queue_pop(&s_queue, out, timeout_ms);
}

void mavlink_reader_stop(void) {
    s_reader.running = false;
    if (s_reader.tid) {
        pthread_join(s_reader.tid, NULL);
        s_reader.tid = 0;
    }
    if (s_reader.fd >= 0) {
        close(s_reader.fd);
        s_reader.fd = -1;
    }
}

bool mavlink_reader_running(void) {
    return s_reader.running;
}
