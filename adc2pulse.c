/*
 *  Odroid SARADC (channel0) volume control
 *
 *  compile:
 *  $ gcc -Wall -Wl,-rpath,/home/user/lib -I/home/user/include -o adc2pulse adc2pulse.c -L/home/user/lib -lpulse -lpthread
 */

#include <sys/select.h>
#include <sys/types.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

#include <pulse/pulseaudio.h>


#define INTERVAL_FAST_SCAN 8000000  // 8ms
#define INTERVAL_SLOW_SCAN 50000000  // 50ms

static pthread_rwlock_t vlock, slock;
static pa_cvolume *vol;  // volume reported by pulseaudio  (vlock)
static double curvol;    // last volume set by main thread (slock)
static double newvol;    // new volume to be set           (slock)
static int state_fd;
static int mute = 0;

static char *sink_name;
static char *state_file;
static char *client_name = "volume-pot";
static int adc_channel = 0;


static void set_volume_cb(pa_context *c, int success, void *userdata) {
    if (!success)
        fprintf(stderr, "failed to set volume: %s\n", pa_strerror(pa_context_errno(c)));
}


static void sink_info_cb(pa_context *c, const pa_sink_info *i, int is_last, void *userdata)
{
    static struct stat st;
    static char state[32];
    static pa_cvolume cv;
    int n, l, v = 0;

    if (is_last)
        return;

    if (strcmp(i->name, sink_name))
        return;

    // save volume and mute state
    pthread_rwlock_wrlock(&vlock);
    mute = i->mute;
    cv = i->volume;
    vol = &cv;
    pthread_rwlock_unlock(&vlock);

    // save state file
    pthread_rwlock_rdlock(&vlock);
    for (n = 0; n < vol->channels; n++)
        v += vol->values[n];
    snprintf(state, sizeof(state), "%d:%d/%d\n", mute ? 1 : 0, v / vol->channels, PA_VOLUME_NORM);
    pthread_rwlock_unlock(&vlock);

    if (fstat(state_fd, &st) == -1)
        return;

    if (st.st_size > sizeof(state))
        return;

    l = strlen(state);

    if (l < st.st_size) {
        memset(state + l, ' ', sizeof(state) - l);
        pwrite(state_fd, state, st.st_size, 0);
        ftruncate(state_fd, l);
    } else
        pwrite(state_fd, state, l, 0);
}


static void subscribe_cb(pa_context *c, pa_subscription_event_type_t t, uint32_t idx, void *userdata)
{
    pa_operation *op = NULL;

    switch (t & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) {
        case PA_SUBSCRIPTION_EVENT_SINK:
            op = pa_context_get_sink_info_by_index(c, idx, sink_info_cb, NULL);

            if (!op) {
                fprintf(stderr, "pa_context_get_sink_info_by_index() failed: %s\n", pa_strerror(pa_context_errno(c)));
                exit(1);
            } else
                pa_operation_unref(op);

            break;
    }
}


static void ctx_state_cb(pa_context *c, void *userdata) {
    pa_operation *op = NULL;

    switch (pa_context_get_state(c)) {
        case PA_CONTEXT_CONNECTING:
        case PA_CONTEXT_AUTHORIZING:
        case PA_CONTEXT_SETTING_NAME:
            break;

        case PA_CONTEXT_READY:
            op = pa_context_get_sink_info_by_name(c, sink_name, sink_info_cb, NULL);

            if (!op) {
                fprintf(stderr, "pa_context_get_sink_info_by_name() failed: %s\n", pa_strerror(pa_context_errno(c)));
                exit(1);
            }

            pa_context_set_subscribe_callback(c, subscribe_cb, NULL);

            op = pa_context_subscribe(c, PA_SUBSCRIPTION_MASK_SINK, NULL, NULL);

            if (!op) {
                fprintf(stderr, "pa_context_subscribe() failed: %s\n", pa_strerror(pa_context_errno(c)));
                exit(1);
            }

            break;

        case PA_CONTEXT_TERMINATED:
            fprintf(stderr, "pulseaudio context terminated\n");
            exit(0);
            break;

        case PA_CONTEXT_FAILED:
        default:
            fprintf(stderr, "pulseaudio connection failure: %s\n", pa_strerror(pa_context_errno(c)));
            exit(1);
    }
}


static int adc_open(unsigned num)
{
    char path[256];
    int fd;

    sprintf(path, "/sys/class/saradc/saradc_ch%u", num);
    fd = open(path, O_RDONLY);
    if (fd == -1) {
        perror("failed to open ADC channel");
        return -1;
    }

    return fd;
}


static int adc_read(int fd)
{
    char buf[16];
    int n;

    n = pread(fd, buf, sizeof(buf) - 1, 0);

    if (n <= 0)
        return 0;

    if (buf[n] == '\n')
        buf[n] = '\0';
    else
        buf[n + 1] = '\0';

    return atoi(buf);
}


void *set_volume_thread(void *userdata)
{
    pa_context *context = userdata;
    static pa_cvolume pavol;
    struct timespec ts;
    pa_operation *op;
    int i;

    for (;;) {
        pthread_rwlock_rdlock(&slock);
        if (newvol == curvol) {
            pthread_rwlock_unlock(&slock);
            goto sleep;
        }
        pthread_rwlock_unlock(&slock);

        // update volume
        pthread_rwlock_rdlock(&vlock);
        pavol = *vol;
        pthread_rwlock_unlock(&vlock);

        pthread_rwlock_wrlock(&slock);
        curvol = newvol;
        for (i = 0; i < pavol.channels; i++)
            if (newvol > 1)
                pavol.values[i] = PA_VOLUME_NORM;
            else
            if (newvol < 0)
                pavol.values[i] = 0;
            else
                pavol.values[i] = newvol * PA_VOLUME_NORM;
        //printf("volume set to: %.02f (pulse)\n", 100.0 * newvol);
        pthread_rwlock_unlock(&slock);

        op = pa_context_set_sink_volume_by_name(context, sink_name, &pavol, set_volume_cb, NULL);
        if (op)
            pa_operation_unref(op);

        // update interval: 50ms
        sleep:
        ts.tv_sec = 0;
        ts.tv_nsec = 50000000;
        nanosleep(&ts, NULL);
    }
}


static void set_volume(double v)
{
    pthread_rwlock_wrlock(&slock);
    newvol = v;
    pthread_rwlock_unlock(&slock);
}


int main(int argc, char *argv[])
{
    char buffer[4096];
    pthread_t vthread;
    pthread_attr_t attr;
    pa_context *context;
    pa_mainloop_api *ml_api;
    pa_threaded_mainloop *ml;
    int i, raw, adc_fd, stable;
    double val, ema, emas, res;
    double k1, k2, k3, k4;
    struct timespec ts;
    unsigned interval;
    double delta;

    // check command line arguments
    if (argc < 3) {
        fprintf(stderr, "Usage:\n %s <sink-name> <state-file>\n", argv[0]);
        return 1;
    }

    sink_name = strdup(argv[1]);
    state_file = strdup(argv[2]);

    if (!sink_name || !state_file) {
        fprintf(stderr, "Cannot allocate mamory\n");
        return 1;
    }

    // open adc
    adc_fd = adc_open(adc_channel);
    if (adc_fd == -1)
        return 1;

    // init locks
    pthread_rwlock_init(&vlock, NULL);
    pthread_rwlock_init(&slock, NULL);

    // open state file
    state_fd = open(state_file, O_CREAT | O_TRUNC | O_WRONLY, 0666);
    if (state_fd == -1) {
        fprintf(stderr, "open state file failed\n");
        return 1;
    }

    // get a mainloop and its context
    ml = pa_threaded_mainloop_new();
    ml_api = pa_threaded_mainloop_get_api(ml);
    context = pa_context_new(ml_api, client_name);

    // set a context callback
    pa_context_set_state_callback(context, ctx_state_cb, NULL);

    // start the mainloop
    if (pa_threaded_mainloop_start(ml) < 0) {
        fprintf(stderr, "pa_threaded_mainloop_run() failed\n");
        return 1;
    }

    // reset volume
    vol = NULL;

    // connect
    if (pa_context_connect(context, NULL, 0, NULL) < 0) {
        fprintf(stderr, "pa_context_connect() failed\n");
        return 1;
    }

    // wait the volume
    for (i = 0; i < 100; i++) {
        pthread_rwlock_rdlock(&vlock);
        if (vol) {
            newvol = curvol = (double) vol->values[0] / PA_VOLUME_NORM;
            pthread_rwlock_unlock(&vlock);
            break;
        }
        pthread_rwlock_unlock(&vlock);

        ts.tv_sec = 0;
        ts.tv_nsec = 50000000;
        nanosleep(&ts, NULL);
    }

    if (i == 100) {
        fprintf(stderr, "Timeout waiting sink state: %s\n", sink_name);
        return 1;
    }

    buffer[0] = '\0';
    for (i = 0; i < vol->channels; i++)
        sprintf(buffer + strlen(buffer), i ? ", %d/%d" : "%d/%d",
                vol->values[i], PA_VOLUME_NORM);
    fprintf(stderr, "connected to pulseaudio, sink: %s, mute: %d, volume: %s\n",
            sink_name, mute, buffer);

    // start volume thread
    if (pthread_attr_init(&attr) != 0) {
        perror("Attribute init failed");
        return 1;
    }

    if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0) {
        perror("Setting detached state failed");
        return 1;
    }

    if (pthread_create(&vthread, &attr, set_volume_thread, context) != 0) {
        perror("Creation of thread failed");
        return 1;
    }

    fprintf(stderr, "starting ADC read loop\n");

    // adc resolution: 10bit -> 0.0 ... 102.0
    res = 1002.9;

    // unstable ema ~ last 7 values
    k1 = 2.0 / 8.0;
    k2 = 1.0 - k1;
    for (raw = 0, i = 0; i < 32; i++)
        raw += adc_read(adc_fd);
    ema = (double) raw / res / 32.0 - 0.01;

    // stable ema ~ last 150 values
    stable = 100000;
    k3 = 2.0 / 151.0;
    k4 = 1.0 - k3;

    // slow scan
    interval = INTERVAL_SLOW_SCAN;

    // adc read loop
    for (;;) {
        // sleep for a while
        ts.tv_sec = 0;
        ts.tv_nsec = interval;
        nanosleep(&ts, NULL);

        // read pot position -> -1.0 ... 101.0
        for (raw = 0, i = 0; i < 16; i++)
            raw += adc_read(adc_fd);
        val = (double) raw / res / 16.0 - 0.01;

        // calculate unstable ema
        ema = k1 * val + k2 * ema;

        // volume delta
        pthread_rwlock_rdlock(&slock);
        delta = ema > newvol ? ema - newvol : newvol - ema;
        pthread_rwlock_unlock(&slock);

        // unstable ema
        if (delta >= 0.01) {
            // set fast scan
            interval = INTERVAL_FAST_SCAN;
            // start stable ema calculation
            stable = 0;
            emas = ema;
            // set unstable volume
            set_volume(ema);
            //printf("volume set to: %.02f\n", 100.0 * ema);
        } else {
            // stable ema calculation
            if (stable < 200) {
                stable++;
                emas = k3 * val + k4 * emas;
            } else
            if (stable == 200) {
                stable++;
                // set slow scan
                interval = INTERVAL_SLOW_SCAN;
                // set stable volume
                set_volume(emas);
                //printf("volume set to: %.02f (final)\n", 100.0 * emas);
            }
        }
    }

    // shut up gcc
    return 0;
}
