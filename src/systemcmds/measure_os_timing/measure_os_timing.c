/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <debug.h>

#include <math.h>
#include <pthread.h>
#include <nuttx/wdog.h>



/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
__EXPORT int measure_os_timing_main(int argc, char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WAKEUP_SIGNAL SIGALRM
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void *mutex_swap_thread_func(void *parameter);
static int mutex_swap_test(void);
static void wd_timeout_func(int argc, uint32_t arg1, ...);
static void *wd_swap_hw_thread_func(void *parameter);
static void *wd_swap_calc_thread_func(void *parameter);
static int wd_swap_test(void);
static void sigalarm(int signo);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static pthread_t t1_pthread = 0;
static pthread_t t2_pthread = 0;
static int thread_run = 0;
static pthread_mutex_t mut;

struct wd_swap_params {
	WDOG_ID wdog;
	int sys_tic_delay;
	double angle;
	double increment;
	double cos_angle;
	pthread_t waiter;
};

static struct wd_swap_params wd_swap_data  = {
	.wdog = (WDOG_ID)-1,
	.sys_tic_delay = 1,
	.angle = 0.0,
	.cos_angle = 0.0,
	.increment = 0.01 * M_DEG_TO_RAD,
	.waiter = -1
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * wd_timeout
 ****************************************************************************/

static void wd_timeout_func(int argc, uint32_t arg1, ...)
{
	  struct wd_swap_params * data  = (struct wd_swap_params *)arg1;
	  union sigval sigvalue;
	  sigvalue.sival_int = 0;

	  PROBE(4,true);
	  int status = sigqueue(data->waiter, WAKEUP_SIGNAL, sigvalue);
	  if (status != OK)
		{
		  printf("wd_timeout: ERROR sigqueue failed\n" );
		}

	  PROBE(4,false);
}

/****************************************************************************
 * wd_swap_hw_thread_func
 ****************************************************************************/
static void sigalarm(int signo)
{
    (void)signo;
}
static void *wd_swap_hw_thread_func(void *parameter)
{
	struct wd_swap_params * data  = (struct wd_swap_params *)parameter;
    printf("Wd hw thread: Start\n");

    sigset_t sigset, oldset;
	sigemptyset(&sigset);
	sigaddset(&sigset, WAKEUP_SIGNAL);
	pthread_sigmask(SIG_UNBLOCK, &sigset, &oldset);

	// Install the signal handler for SIGINT.
	struct sigaction s;
	s.sa_handler = sigalarm;
	sigemptyset(&s.sa_mask);
	s.sa_flags = 0;
	sigaction(WAKEUP_SIGNAL, &s, NULL);

	data->waiter = getpid();

    /* Set initial timer */
    wd_start(data->wdog, data->sys_tic_delay, wd_timeout_func, 1, data);

	while(thread_run) {



		/* Wait on Timer */

		PROBE(3,false);
		pause();
		PROBE(3,true);
		/* Set next timer */

		wd_start(data->wdog, data->sys_tic_delay, wd_timeout_func, 1, data);
	}

	printf("Wd hw thread: Exit\n");
	wd_cancel(data->wdog);
	pthread_exit(NULL);

}
/****************************************************************************
 * wd_swap_waiter_thread_func
 ****************************************************************************/

static void *wd_swap_calc_thread_func(void *parameter)
{
	struct wd_swap_params * data  = (struct wd_swap_params *)parameter;
    printf("Wd calc thread: Start\n");

	while(thread_run) {
		PROBE(5,true);
//	    data->cos_angle = cos(data->angle);
//		data->angle += data->increment;
		PROBE(5,false);
	}
    printf("Wd calc thread: Exit\n");
	pthread_exit(NULL);
}

/****************************************************************************
 * wd_swap_test
 ****************************************************************************/

static int wd_swap_test(void)
{
	struct wd_swap_params *data =  &wd_swap_data;

	data->wdog = wd_create();
	if (!data->wdog) {
	  fdbg("ERROR: Wd thread Failed to create a timer\n");
	  return -ENOMEM;
	}

	int policy;
	int last_priority = 0; // Priority we started with
	struct sched_param param;

	/* Save and boost Priority */
	pthread_getschedparam(0, &policy,&param);
	last_priority = param.sched_priority;
	param.sched_priority = SCHED_PRIORITY_MAX -5;
	pthread_setschedparam(0, policy,&param);
	printf("wd_swap_test Boosted Priority from %d to %d\n",last_priority,param.sched_priority);

	thread_run = 1;

	pthread_attr_t t1_atts;
	pthread_attr_t t2_atts;
	pthread_attr_init(&t1_atts);
	pthread_attr_init(&t2_atts);

	pthread_attr_setstacksize(&t1_atts, 1024);
	pthread_attr_setstacksize(&t2_atts, 2048);

	param.sched_priority = SCHED_PRIORITY_MAX -20;
	(void)pthread_attr_setschedparam(&t1_atts, &param);

	param.sched_priority = SCHED_PRIORITY_MAX -10;
	(void)pthread_attr_setschedparam(&t2_atts, &param);


	if (0 != pthread_create(&t1_pthread, &t1_atts, wd_swap_calc_thread_func ,(pthread_addr_t)data)) {
		printf("wd_swap_test ERROR: creating thread 1\n");
	}
	if (0 != pthread_create(&t2_pthread, &t2_atts, wd_swap_hw_thread_func,(pthread_addr_t)data)) {
		printf("wd_swap_test ERROR: creating thread 2\n");
	}
	printf("wd_swap_test wait\n");
	sleep(10);
	printf("wd_swap_test request threads exit\n");
	thread_run = 0;

	pthread_kill(t2_pthread,WAKEUP_SIGNAL);

	pthread_getschedparam(0, &policy,&param);
	param.sched_priority = last_priority;
	pthread_setschedparam(0, policy,&param);
	printf("wd_swap_test Dropped Priority\n");

	pthread_join(t1_pthread, NULL);
	pthread_join(t2_pthread, NULL);
	printf("wd_swap_test done\n");
	wd_delete(data->wdog);
	data->wdog= (WDOG_ID)-1;
	data->angle = 0.0;
	data->cos_angle = 0.0;
	return 0;

}

/****************************************************************************
 * mutex_swap_thread_func
 ****************************************************************************/

static void *mutex_swap_thread_func(void *parameter)
{
	int id  = (int)parameter;
    printf("Mutex thread %d: Start\n", id);

	while(thread_run) {

		if (id == 1) {PROBE(3,false);} else {PROBE(4,false);}
		int status = pthread_mutex_lock(&mut);
		if (id == 1) {PROBE(3,true);} else {PROBE(4,true);}

		if (status != 0) {
			  printf("ERROR Mutex thread %d: pthread_mutex_lock failed, status=%d\n",
					  id, status);
		}

		if (id == 1) {PROBE_MARK(3);} else {PROBE_MARK(4);}
		pthread_yield();
		if (id == 1) {PROBE_MARK(3);} else {PROBE_MARK(4);}
		 status = pthread_mutex_unlock(&mut);
		 if (status != 0) {
			  printf("ERROR Mutex thread %d: pthread_mutex_unlock failed, status=%d\n",
					  id, status);
		 }
	}
	printf("Mutex thread %d: Exit\n", id);
	pthread_exit(NULL);
}

/****************************************************************************
 * mutex_swap_test
 ****************************************************************************/

static int mutex_swap_test(void)
{

	int policy;
	int last_priority = 0; // Priority we started with
	struct sched_param param;

	/* Save and boost Priority */
	pthread_getschedparam(0, &policy,&param);
	last_priority = param.sched_priority;
	param.sched_priority = SCHED_PRIORITY_MAX -5;
	pthread_setschedparam(0, policy,&param);
    printf("mutex_swap Boosted Priority from %d to %d\n",last_priority,param.sched_priority);

	thread_run = 1;

	pthread_mutex_init(&mut, NULL);

	pthread_attr_t t1_atts;
	pthread_attr_t t2_atts;
	pthread_attr_init(&t1_atts);
	pthread_attr_init(&t2_atts);


	param.sched_priority = SCHED_PRIORITY_MAX -10;

	(void)pthread_attr_setschedparam(&t1_atts, &param);
	(void)pthread_attr_setschedparam(&t2_atts, &param);

	pthread_attr_setstacksize(&t1_atts, 1024);
	pthread_attr_setstacksize(&t2_atts, 1024);

	if (0 != pthread_create(&t1_pthread, &t1_atts, mutex_swap_thread_func,(pthread_addr_t)1)) {
		printf("mutex_swap ERROR: creating thread 1\n");
	}
	if (0 != pthread_create(&t2_pthread, &t2_atts, mutex_swap_thread_func,(pthread_addr_t)2)) {
		printf("mutex_swap ERROR creating thread 2\n");
	}
	printf("mutex_swap wait\n");
	sleep(10);
	printf("mutex_swap request threads exit\n");
	thread_run = 0;

	pthread_getschedparam(0, &policy,&param);
	param.sched_priority = last_priority;
	pthread_setschedparam(0, policy,&param);
    printf("mutex_swap Dropped Priority\n");

	pthread_join(t1_pthread, NULL);
	pthread_join(t2_pthread, NULL);
	printf("mutex_swap done\n");
	return 0;

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_os_timing
 ****************************************************************************/

int measure_os_timing_main(int argc, char *argv[])
{


	PROBE(6,true);
	PROBE(6,false);

	/*
	 * Measure the wd_ context swap time.
	 */
	if (!strcmp(argv[1], "wd"))
		return wd_swap_test();

	/*
	 * Measure the mutex context swap time.
	 */
	if (!strcmp(argv[1], "mutex"))
		return mutex_swap_test();

	fprintf(stderr, "unrecognised command, try 'wd', 'mutex' \n");
	return -EINVAL;

}
