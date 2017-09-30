/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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

/**
 * @file loadtest.c
 */

#include <px4_config.h>
#include <px4_log.h>
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <poll.h>

__EXPORT int loadtest_main(int argc, char *argv[]);


static bool is_prime(int number)
{
	bool isprime = true;

	if (number == 2)	{
		return isprime;

	} else if (number <= 1 || number % 2 == 0) {

		return !isprime;

	} else {

		const int to = sqrt(number);

		for (int i = 3; i <= to; i += 2)
			if (!(isprime = number % i)) {
				break;
			}

		return isprime;
	}

	return false;
}


static bool is_mersenne_prime(int number)
{
	const long long unsigned mersenne_prime = (1LLU << number) - 1;
	long long unsigned s = 4;

	if (number != 2) {

		for (int i = 3; i <= number; i++) {
			s = (s * s - 2) % mersenne_prime;
		}

		return s == 0;
	}

	return true;
}


int loadtest_main(int argc, char **argv)
{

	const int upper_bound = log2l(ULLONG_MAX) / 2;

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	PX4_INFO("Heating up the CPU Press CTRL-C or 'c' to abort.");

	while (1) {

		for (int number = 2; number <= upper_bound; number += 1) {
			if (is_prime(number) && is_mersenne_prime(number)) {
				printf("M%u ", number);
			}
		}

		printf("\n");

		/* abort on user request */

		int ret = poll(&fds, 1, 0);

		if (ret > 0) {

			char c;
			ret = read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {

				PX4_INFO("User abort\n");
				return 0;
			}
		}
	}

	return 0;
}
