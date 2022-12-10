// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "string.h"
#include "trace.h"
#include "serial_if.h"
#include "serial_ll_if.h"
#include "platform_wrapper.h"

#include "swift_timer.h"

#define MILLISEC_TO_SEC			1000
#define SEC_TO_MILLISEC(x) (1000*(x))

#define HOSTED_CALLOC(buff,nbytes) do {                           \
    buff = (uint8_t *)hosted_calloc(1, nbytes);                   \
    if (!buff) {                                                  \
        printf("%s, Failed to allocate memory \n", __func__);     \
        goto free_bufs;                                           \
    }                                                             \
} while(0);


static void *read_sem;
static serial_ll_handle_t * serial_ll_if_g;

static void control_path_rx_indication(void);

struct serial_drv_handle_t {
	int handle; /* dummy variable */
};


int control_path_platform_init(void)
{
	/* control path semaphore */
	read_sem = swifthal_os_sem_create(0, 1);
	assert(read_sem);

	/* grab the semaphore, so that task will be mandated to wait on semaphore */
	//swifthal_os_sem_take(read_sem, -1);

	serial_ll_if_g = serial_ll_init(control_path_rx_indication);
	if (!serial_ll_if_g) {
		printf("Serial interface creation failed\n\r");
		assert(serial_ll_if_g);
		return SWIFTIO_FAIL;
	}
	if (SWIFTIO_OK != serial_ll_if_g->fops->open(serial_ll_if_g)) {
		printf("Serial interface open failed\n\r");
		return SWIFTIO_FAIL;
	}
	return SWIFTIO_OK;
}

int control_path_platform_deinit(void)
{
	if (SWIFTIO_OK != serial_ll_if_g->fops->close(serial_ll_if_g)) {
		printf("Serial interface close failed\n\r");
		return SWIFTIO_FAIL;
	}
	return SWIFTIO_OK;
}

static void control_path_rx_indication(void)
{
	/* heads up to control path for read */
	if(read_sem) {
		swifthal_os_sem_give(read_sem);
	}
}

/* -------- Memory ---------- */
void* hosted_malloc(size_t size)
{
	return malloc(size);
}

void* hosted_calloc(size_t blk_no, size_t size)
{
	void* ptr = malloc(blk_no*size);
	if (!ptr) {
		return NULL;
	}

	memset(ptr, 0, blk_no*size);
	return ptr;
}

void hosted_free(void* ptr)
{
	if(ptr) {
		free(ptr);
		ptr=NULL;
	}
}

void *hosted_realloc(void *mem, size_t newsize)
{
	void *p = NULL;

	if (newsize == 0) {
		mem_free(mem);
		return NULL;
	}

	p = hosted_malloc(newsize);
	if (p) {
		/* zero the memory */
		if (mem != NULL) {
			memcpy(p, mem, newsize);
			mem_free(mem);
		}
	}
	return p;
}

/* -------- Threads ---------- */
void *hosted_thread_create(void (*start_routine)(void const *), void *arg)
{
	if (!start_routine) {
		printf("start_routine is mandatory for thread create\n");
		return NULL;
	}

	return swifthal_os_task_create("CTL_TASK",
							start_routine,
							arg, NULL, NULL,
							CTRL_PATH_TASK_PRIO,
							CTRL_PATH_TASK_STACK_SIZE);
}

int hosted_thread_cancel(void *thread_handle)
{
	printf("No support cancel task");

	return SWIFTIO_OK;
}

/* -------- Semaphores ---------- */
void * hosted_create_semaphore(int init_value)
{
	return swifthal_os_sem_create(init_value, 10000);
}

unsigned int e_sleep(unsigned int seconds) {
   swifthal_ms_sleep(seconds * 1000);
   return 0;
}

unsigned int msleep(unsigned int mseconds) {
   swifthal_ms_sleep(mseconds);
   return 0;
}

int hosted_get_semaphore(void * semaphore_handle, int timeout)
{

	if (!timeout) {
		/* non blocking */
		return swifthal_os_sem_take(semaphore_handle, 0);
	} else if (timeout<0) {
		/* Blocking */
		return swifthal_os_sem_take(semaphore_handle, -1);
	} else {
		return swifthal_os_sem_take(semaphore_handle, SEC_TO_MILLISEC(timeout));
	}
}

int hosted_post_semaphore(void * semaphore_handle)
{
	return swifthal_os_sem_give(semaphore_handle);
}

int hosted_destroy_semaphore(void * semaphore_handle)
{
	return swifthal_os_sem_destroy(semaphore_handle);
}
/* -------- Timers  ---------- */
int hosted_timer_stop(void *timer_handle)
{
	int ret = SWIFTIO_OK;

	swifthal_timer_stop(timer_handle);
	return swifthal_timer_close(timer_handle);
}

/* Sample timer_handler looks like this:
 *
 * void expired(union sigval timer_data){
 *     struct mystruct *a = timer_data.sival_ptr;
 * 	printf("Expired %u\n", a->mydata++);
 * }
 **/

void *hosted_timer_start(int duration, int type,
		void (*timeout_handler)(void const *), void *arg)
{
	swift_timer_type_t swift_type;
	void *timer = swifthal_timer_open();
	swifthal_timer_add_callback(timer, arg, timeout_handler);
	
	if(timer == NULL){
		return NULL;
	}

	/* timer type */
	if (type == CTRL__TIMER_PERIODIC) {
		swift_type = SWIFT_TIMER_TYPE_PERIOD;
	} else if (type == CTRL__TIMER_ONESHOT) {
		swift_type = SWIFT_TIMER_TYPE_ONESHOT;
	} else {
		printf("Unsupported timer type. supported: one_shot, periodic\n");
		swifthal_timer_close(timer);
		return NULL;
	}

	swifthal_timer_start(timer, swift_type, SEC_TO_MILLISEC(duration));

	return timer;
}

/* -------- Serial Drv ---------- */
struct serial_drv_handle_t* serial_drv_open(const char *transport)
{
	struct serial_drv_handle_t* serial_drv_handle = NULL;
	if (!transport) {
		printf("Invalid parameter in open \n\r");
		return NULL;
	}

	if(serial_drv_handle) {
		printf("return orig hndl\n");
		return serial_drv_handle;
	}

	serial_drv_handle = (struct serial_drv_handle_t*) hosted_calloc
		(1,sizeof(struct serial_drv_handle_t));
	if (!serial_drv_handle) {
		printf("Failed to allocate memory \n");
		return NULL;
	}

	return serial_drv_handle;
}

int serial_drv_write (struct serial_drv_handle_t* serial_drv_handle,
	uint8_t* buf, int in_count, int* out_count)
{
	int ret = 0;
	if (!serial_drv_handle || !buf || !in_count || !out_count) {
		printf("Invalid parameters in write\n\r");
		return SWIFTIO_FAIL;
	}

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->write)) {
		printf("serial interface not valid\n\r");
		return SWIFTIO_FAIL;
	}

	ret = serial_ll_if_g->fops->write(serial_ll_if_g, buf, in_count);
	if (ret != SWIFTIO_OK) {
		*out_count = 0;
		printf("Failed to write data\n\r");
		return SWIFTIO_FAIL;
	}

	*out_count = in_count;
	return SWIFTIO_OK;
}


uint8_t * serial_drv_read(struct serial_drv_handle_t *serial_drv_handle,
		uint32_t *out_nbyte)
{
	uint16_t init_read_len = 0;
	uint16_t rx_buf_len = 0;
	uint8_t* read_buf = NULL;
	int ret = 0;
	/* Any of `CTRL_EP_NAME_EVENT` and `CTRL_EP_NAME_RESP` could be used,
	 * as both have same strlen in adapter.h */
	const char* ep_name = CTRL_EP_NAME_RESP;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;


	if (!serial_drv_handle || !out_nbyte) {
		printf("Invalid parameters in read\n\r");
		return NULL;
	}

	*out_nbyte = 0;

	if(!read_sem) {
		printf("Semaphore not initialized\n\r");
		return NULL;
	}
	if (swifthal_os_sem_take(read_sem, -1) != 0) {
		printf("Failed to read data \n\r");
		return NULL;
	}

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->read)) {
		printf("serial interface refusing to read\n\r");
		return NULL;
	}

	/* Get buffer from serial interface */
	read_buf = serial_ll_if_g->fops->read(serial_ll_if_g, &rx_buf_len);
	if ((!read_buf) || (!rx_buf_len)) {
		printf("serial read failed\n\r");
		return NULL;
	}
	print_hex_dump(read_buf, rx_buf_len, "Serial read data");

/*
 * Read Operation happens in two steps because total read length is unkown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * (1) Read fixed length of RX data :
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 *
 *  int_read_len = 1 + 2 + Endpoint length + 1 + 2
 */

	init_read_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) +
		SIZE_OF_TYPE + SIZE_OF_LENGTH;

	if(rx_buf_len < init_read_len) {
		mem_free(read_buf);
		printf("Incomplete serial buff, return\n");
		return NULL;
	}

	HOSTED_CALLOC(buf,init_read_len);

	memcpy(buf, read_buf, init_read_len);

	/* parse_tlv function returns variable payload length
	 * of received data in buf_len
	 **/
	ret = parse_tlv(buf, &buf_len);
	if (ret || !buf_len) {
		mem_free(buf);
		printf("Failed to parse RX data \n\r");
		goto free_bufs;
	}

	if (rx_buf_len < (init_read_len + buf_len)) {
		printf("Buf read on serial iface is smaller than expected len\n");
		mem_free(buf);
		goto free_bufs;
	}

	mem_free(buf);
/*
 * (2) Read variable length of RX data:
 */
	HOSTED_CALLOC(buf,buf_len);

	memcpy((buf), read_buf+init_read_len, buf_len);

	mem_free(read_buf);

	*out_nbyte = buf_len;
	return buf;

free_bufs:
	mem_free(read_buf);
	mem_free(buf);
	return NULL;
}

int serial_drv_close(struct serial_drv_handle_t** serial_drv_handle)
{
	if (!serial_drv_handle || !(*serial_drv_handle)) {
		printf("Invalid parameter in close \n\r");
		if (serial_drv_handle)
			mem_free(serial_drv_handle);
		return SWIFTIO_FAIL;
	}
	mem_free(*serial_drv_handle);
	return SWIFTIO_OK;
}
