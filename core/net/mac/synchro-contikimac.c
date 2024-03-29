/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Implementation of the ContikiMAC power-saving radio duty cycling protocol
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

/* Modified by RMonica
 *
 * Patches: - different nodes may have different cycle times
 *          - add RPL function RPL_DAG_MC_AVG_DELAY
 *          - more efficent way to calculate wakeup time
 */

#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/mac/contikimac.h"
#include "net/netstack.h"
#include "net/rime.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
// added by Pietro
#include "net/rpl/rpl.h"
#include "net/uip-ds6.h"
#include "net/rime/rimeaddr.h"
#include "dev/cc2420.h"


#include <string.h>

/* TX/RX cycles are synchronized with neighbor wake periods */
#ifndef WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION      1
#endif
/* Two byte header added to allow recovery of padded short packets */
/* Wireshark will not understand such packets at present */
#ifdef CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER
#define WITH_CONTIKIMAC_HEADER       CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER
#else
#define WITH_CONTIKIMAC_HEADER       1
#endif
/* More aggressive radio sleeping when channel is busy with other traffic */
#ifndef WITH_FAST_SLEEP
#define WITH_FAST_SLEEP              1
#endif
/* Radio does CSMA and autobackoff */
#ifndef RDC_CONF_HARDWARE_CSMA
#define RDC_CONF_HARDWARE_CSMA       0
#endif
/* Radio returns TX_OK/TX_NOACK after autoack wait */
#ifndef RDC_CONF_HARDWARE_ACK
#define RDC_CONF_HARDWARE_ACK        0
#endif
/* MCU can sleep during radio off */
#ifndef RDC_CONF_MCU_SLEEP
#define RDC_CONF_MCU_SLEEP           0
#endif

#if WITH_CONTIKIMAC_HEADER
#define CONTIKIMAC_ID 0x00

struct hdr {
  uint8_t id;
  uint8_t len;
};
#endif /* WITH_CONTIKIMAC_HEADER */

/* If RTIMER_ARCH_SECOND is not a multiple of CYCLE_TIME, there will be an inexact
 * number of channel checks per second due to the truncation of CYCLE_TIME.
 * This will degrade the effectiveness of phase optimization with neighbors that
 * do not have the same truncation error.
 * Define SYNC_CYCLE_STARTS to ensure an integral number of checks per second.
 */
#if (RTIMER_ARCH_SECOND % CYCLE_TIME) != 0
#define SYNC_CYCLE_STARTS                    1
#endif
/* Modified by RMonica
 *
 * CYCLE_TIME_SYNC_TICKS is the remaining of the division of RTIMER_ARCH_SECOND by
 * CYCLE_RATE. This is used by the patch "more efficent way to calculate wakeup time"
 * to re-sync after every second.
 * we expect this number to be very small in most cases (it's always lower than CYCLE_RATE)
 * otherwise, it may be better to set PRECISE_SYNC_CYCLE_STARTS to 1
 */
#define CYCLE_TIME_SYNC_TICKS (RTIMER_ARCH_SECOND - (CYCLE_TIME * CYCLE_RATE))
/* if the following define is 0, there may be a slight imprecision (of at most CYCLE_TIME_SYNC_TICKS ticks) in
 * calculations, but they will be faster.
 * if 1, the patch "more efficent way to calculate wakeup time" will be disabled.
 */
#ifndef PRECISE_SYNC_CYCLE_STARTS
#define PRECISE_SYNC_CYCLE_STARTS 0
#endif

/* Are we currently receiving a burst? */
static int we_are_receiving_burst = 0;

/* BURST_RECV_TIME is the maximum time a receiver waits for the
   next packet of a burst when FRAME_PENDING is set. */
#define INTER_PACKET_DEADLINE               CLOCK_SECOND / 32

/* ContikiMAC performs periodic channel checks. Each channel check
   consists of two or more CCA checks. CCA_COUNT_MAX is the number of
   CCAs to be done for each periodic channel check. The default is
   two.*/
#define CCA_COUNT_MAX                      2

/* Before starting a transmission, Contikimac checks the availability
   of the channel with CCA_COUNT_MAX_TX consecutive CCAs */
#define CCA_COUNT_MAX_TX                   6

/* CCA_CHECK_TIME is the time it takes to perform a CCA check. */
/* Note this may be zero. AVRs have 7612 ticks/sec, but block until cca is done */
#define CCA_CHECK_TIME                     RTIMER_ARCH_SECOND / 8192

/* CCA_SLEEP_TIME is the time between two successive CCA checks. */
/* Add 1 when rtimer ticks are coarse */
#if RTIMER_ARCH_SECOND > 8000
#define CCA_SLEEP_TIME                     RTIMER_ARCH_SECOND / 2000
#else
#define CCA_SLEEP_TIME                     (RTIMER_ARCH_SECOND / 2000) + 1
#endif

/* CHECK_TIME is the total time it takes to perform CCA_COUNT_MAX
   CCAs. */
#define CHECK_TIME                         (CCA_COUNT_MAX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* CHECK_TIME_TX is the total time it takes to perform CCA_COUNT_MAX_TX
   CCAs. */
#define CHECK_TIME_TX                      (CCA_COUNT_MAX_TX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* LISTEN_TIME_AFTER_PACKET_DETECTED is the time that we keep checking
   for activity after a potential packet has been detected by a CCA
   check. */
#define LISTEN_TIME_AFTER_PACKET_DETECTED  RTIMER_ARCH_SECOND / 80

/* MAX_SILENCE_PERIODS is the maximum amount of periods (a period is
   CCA_CHECK_TIME + CCA_SLEEP_TIME) that we allow to be silent before
   we turn of the radio. */
#define MAX_SILENCE_PERIODS                5

/* MAX_NONACTIVITY_PERIODS is the maximum number of periods we allow
   the radio to be turned on without any packet being received, when
   WITH_FAST_SLEEP is enabled. */
#define MAX_NONACTIVITY_PERIODS            10



/* STROBE_TIME is the maximum amount of time a transmitted packet
   should be repeatedly transmitted as part of a transmission. */
/* Modified by RMonica
 *
 * since multiple cycle times are supported, STROBE_TIME must be based
 * on MAX_CYCLE_TIME (see netstack.h) instead of CYCLE_TIME of the current node
 * otherwise, nodes with higher CYCLE_TIME may not receive packets
 * (both unicast and broadcast) because the sender didn't repeated them long enough
 */
#define STROBE_TIME                        (MAX_CYCLE_TIME + 2 * CHECK_TIME)

/* GUARD_TIME is the time before the expected phase of a neighbor that
   a transmitted should begin transmitting packets. */
#define GUARD_TIME                         (10 * CHECK_TIME + CHECK_TIME_TX)

/* INTER_PACKET_INTERVAL is the interval between two successive packet transmissions */
#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 5000

/* AFTER_ACK_DETECTECT_WAIT_TIME is the time to wait after a potential
   ACK packet has been detected until we can read it out from the
   radio. */
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1500

/* MAX_PHASE_STROBE_TIME is the time that we transmit repeated packets
   to a neighbor for which we have a phase lock. */
#define MAX_PHASE_STROBE_TIME              RTIMER_ARCH_SECOND / 60

#ifndef OPTIMIZE_TO_SINK
#define OPTIMIZE_TO_SINK (0)
#endif

#ifndef BOTH_RAWMAC_LADDERS
#define BOTH_RAWMAC_LADDERS (0)
#endif

#define MULTIPHASE_ATTEMPTS (10)

/* Pietro: PHASE_OFFSET should be greater or equal than 2*GUARD_TIME*/
#define PHASE_OFFSET	(2*GUARD_TIME + GUARD_TIME / 4)
/* SHORTEST_PACKET_SIZE is the shortest packet that ContikiMAC
   allows. Packets have to be a certain size to be able to be detected
   by two consecutive CCA checks, and here is where we define this
   shortest size.
   Padded packets will have the wrong ipv6 checksum unless CONTIKIMAC_HEADER
   is used (on both sides) and the receiver will ignore them.
   With no header, reduce to transmit a proper multicast RPL DIS. */
#ifdef CONTIKIMAC_CONF_SHORTEST_PACKET_SIZE
#define SHORTEST_PACKET_SIZE  CONTIKIMAC_CONF_SHORTEST_PACKET_SIZE
#else
#define SHORTEST_PACKET_SIZE               43
#endif

#define MIN_PHASE_OFFSET_FOR_MULTIPHASE (GUARD_TIME)

#define ACK_LEN 3

#include <stdio.h>
static struct rtimer rt;
static struct pt pt;

static volatile uint8_t contikimac_is_on = 0;
static volatile uint8_t contikimac_keep_radio_on = 0;

static volatile unsigned char we_are_sending = 0;
static volatile unsigned char radio_is_on = 0;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#if CONTIKIMAC_CONF_COMPOWER
static struct compower_activity current_packet;
#endif /* CONTIKIMAC_CONF_COMPOWER */

#if WITH_PHASE_OPTIMIZATION

#include "net/mac/phase.h"

#ifdef CONTIKIMAC_CONF_MAX_PHASE_NEIGHBORS
#define MAX_PHASE_NEIGHBORS CONTIKIMAC_CONF_MAX_PHASE_NEIGHBORS
#endif

#ifndef MAX_PHASE_NEIGHBORS
#define MAX_PHASE_NEIGHBORS 30
#endif

PHASE_LIST(phase_list, MAX_PHASE_NEIGHBORS);

#endif /* WITH_PHASE_OPTIMIZATION */

#define DEFAULT_STREAM_TIME (4 * CYCLE_TIME)

#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */

struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 16
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
static struct seqno received_seqnos[MAX_SEQNOS];

#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
static struct timer broadcast_rate_timer;
static int broadcast_rate_counter;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */

static int turn_on(void);
static int turn_off(int);

/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(contikimac_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  if(contikimac_is_on && radio_is_on != 0 &&
     contikimac_keep_radio_on == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static volatile rtimer_clock_t cycle_start;
static char powercycle(struct rtimer *t, void *ptr);
static void
schedule_powercycle(struct rtimer *t, rtimer_clock_t time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(RTIMER_TIME(t) + time, RTIMER_NOW() + 2)) {
      time = RTIMER_NOW() - RTIMER_TIME(t) + 2;
    }

    r = rtimer_set(t, RTIMER_TIME(t) + time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
    	PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_powercycle_fixed(struct rtimer *t, rtimer_clock_t fixed_time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(fixed_time, RTIMER_NOW() + 1)) {
      fixed_time = RTIMER_NOW() + 1;
    }

    r = rtimer_set(t, fixed_time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_powercycle_fixed_move(struct rtimer *t, rtimer_clock_t fixed_time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(fixed_time, RTIMER_NOW() + 1)) {
      fixed_time = RTIMER_NOW() + 1;
    }

    r = rtimer_move(t, fixed_time, 1,
                    (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_off(void)
{
#if CONTIKIMAC_CONF_COMPOWER
  uint8_t was_on = radio_is_on;
#endif /* CONTIKIMAC_CONF_COMPOWER */
  
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    off();
#if CONTIKIMAC_CONF_COMPOWER
    if(was_on && !radio_is_on) {
      compower_accumulate(&compower_idle_activity);
    }
#endif /* CONTIKIMAC_CONF_COMPOWER */
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_on(void)
{
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    on();
  }
}
/*---------------------------------------------------------------------------*/
#define MAX_CYCLE_STARTS 5
typedef struct
{
  rtimer_clock_t offset;
  uint16_t addr;
  uint8_t attempts; // if 0, will never be removed
  uint8_t delay;
} cycle_start_t;
static cycle_start_t cycle_starts[MAX_CYCLE_STARTS];
static uint8_t cycle_starts_size; // size of cycle_starts
static uint8_t cycle_starts_current; // next wakeup index
static uint8_t cycle_prev_rank;
static uint8_t first_cycle_start;
static uint8_t second_cycle_start;
rtimer_clock_t next_cycle_start;
volatile static uint8_t cycle_starts_mutex;
static void swap_cycle_starts(cycle_start_t * a,cycle_start_t * b)
{
  cycle_start_t temp;
  temp = *a;
  *a = *b;
  *b = temp;
}

static void reorder_cycle_starts(rtimer_clock_t current_time)
{
  uint8_t i,h;
  for (i = 0; i < cycle_starts_size - 1; i++)
    for (h = i + 1; h < cycle_starts_size; h++)
      if (cycle_starts[i].offset > cycle_starts[h].offset)
      {
        swap_cycle_starts(&(cycle_starts[i]),&(cycle_starts[h]));

        if (first_cycle_start == i)
          first_cycle_start = h;
        else if (first_cycle_start == h)
          first_cycle_start = i;

        if (second_cycle_start == i)
          second_cycle_start = h;
        else if (second_cycle_start == h)
          second_cycle_start = i;
      }

  rtimer_clock_t current_time_offset = current_time % CYCLE_TIME;
  cycle_starts_current = -1;
  for (i = 0; i < cycle_starts_size; i++)
    if (current_time_offset + CHECK_TIME * 4 < cycle_starts[i].offset)
    {
      cycle_starts_current = i;
      break;
    }
  rtimer_clock_t addendum = 0;
  if (cycle_starts_current == (uint8_t)(-1))
  {
    cycle_starts_current = 0;
    addendum = CYCLE_TIME;
  }
  rtimer_clock_t maybe_next_wakeup = current_time - current_time_offset + addendum + cycle_starts[cycle_starts_current].offset;

  //if (!cycle_starts[cycle_starts_current].delay &&
  //    RTIMER_CLOCK_LT(current_time, maybe_next_wakeup) &&
  //    RTIMER_CLOCK_LT(maybe_next_wakeup + GUARD_TIME, next_cycle_start))
    schedule_powercycle_fixed_move(&rt, maybe_next_wakeup);
}

static void insert_new_cycle_start(rtimer_clock_t offset, uint8_t delay, uint8_t attempts, uint16_t addr,
                                   rtimer_clock_t current_time)
{
  if (cycle_starts_size >= MAX_CYCLE_STARTS)
    return;
  cycle_starts[cycle_starts_size].addr = addr;
  cycle_starts[cycle_starts_size].offset = offset;
  cycle_starts[cycle_starts_size].delay = delay;
  cycle_starts[cycle_starts_size].attempts = attempts;
  //printf("Insert: %u, %u, %u, %u\n",(unsigned)addr,(unsigned)offset,(unsigned)delay,(unsigned)attempts,(unsigned)(current_time));
  if (addr == 0 && attempts == 1)
    second_cycle_start = cycle_starts_size;
  cycle_starts_size++;
  reorder_cycle_starts(current_time);
}

static void remove_cycle_start(rtimer_clock_t current_time, uint16_t addr)
{
  uint8_t i;
  //printf("remove: %u\n",(unsigned)addr);
  for (i = 0; i < cycle_starts_size; i++)
    if (cycle_starts[i].addr == addr)
    {
      swap_cycle_starts(&(cycle_starts[i]),&(cycle_starts[cycle_starts_size - 1]));

      if (first_cycle_start == cycle_starts_size - 1)
        first_cycle_start = i;
      if (second_cycle_start == cycle_starts_size - 1)
        second_cycle_start = i;

      cycle_starts_size--;
      reorder_cycle_starts(current_time);
      break;
    }
}

static void synchro_contikimac_set_multiphase_offset(rtimer_clock_t offset, uint16_t addr,
                                                     unsigned int delay, unsigned int attempts)
{
  if (!attempts)
    return;

  if ((offset < MIN_PHASE_OFFSET_FOR_MULTIPHASE) || (CYCLE_TIME - offset < MIN_PHASE_OFFSET_FOR_MULTIPHASE)) {
    return;
  }

  if (cycle_starts_mutex)
    return;

  cycle_starts_mutex = 1;

  rtimer_clock_t now = RTIMER_NOW();

  offset += cycle_starts[first_cycle_start].offset;
  offset = offset % CYCLE_TIME;
  insert_new_cycle_start(offset, delay, attempts, addr, now);

  cycle_starts_mutex = 0;
}

void synchro_contikimac_schedule_from_metric(unsigned int metric,uint16_t addr)
{
  if (!metric)
    return;
  long unsigned int travel_delay = (long unsigned int)(metric) * (PHASE_OFFSET * 2) + 2 * GUARD_TIME;
  unsigned int cycle_offset = travel_delay / CYCLE_TIME;
  unsigned int offset = travel_delay % CYCLE_TIME;
  unsigned int attempts = MULTIPHASE_ATTEMPTS; // compute this from metric somehow?
  synchro_contikimac_set_multiphase_offset(offset,addr,cycle_offset,attempts);
}

void synchro_contikimac_unschedule_from_metric(uint16_t addr)
{
  if (cycle_starts_mutex)
    return;

  cycle_starts_mutex = 1;
  rtimer_clock_t now = RTIMER_NOW();
  remove_cycle_start(now, addr);
  cycle_starts_mutex = 0;
}

/*---------------------------------------------------------------------------*/
/* Function modified by RMonica
 * for patch "more efficent way to calculate wakeup time"
 */
static char
powercycle(struct rtimer *t, void *ptr)
{
#if SYNC_CYCLE_STARTS
#if PRECISE_SYNC_CYCLE_STARTS
  static volatile rtimer_clock_t sync_cycle_start;
#endif
  static volatile uint8_t sync_cycle_phase;
#endif

  if (cycle_starts_mutex)
  {
    schedule_powercycle_fixed(t,RTIMER_NOW() + CCA_SLEEP_TIME);
    return 0;
  }

  PT_BEGIN(&pt);

#if SYNC_CYCLE_STARTS
#if PRECISE_SYNC_CYCLE_STARTS
  sync_cycle_start = RTIMER_NOW();
#endif
  sync_cycle_phase = 0;
#endif

#if !(SYNC_CYCLE_STARTS && PRECISE_SYNC_CYCLE_STARTS)
  cycle_start = RTIMER_NOW();
  PRINTF("contikimac: cycle_start initial %u\n", cycle_start);
#endif
  cycle_starts_current = 0;
  cycle_starts_size = 1;
  cycle_starts[0].offset = cycle_start % CYCLE_TIME;
  cycle_starts[0].addr = 0;
  cycle_starts[0].attempts = 0;
  cycle_starts[0].delay = 0;
  first_cycle_start = 0;
  second_cycle_start = -1;
  cycle_prev_rank = 0;
  next_cycle_start = cycle_start;
  cycle_start = CYCLE_TIME;

  while(1) {
    static uint8_t packet_seen;
    static rtimer_clock_t t0;
    static uint8_t count;

    if (cycle_starts[cycle_starts_current].addr)
    {
      if (cycle_starts[cycle_starts_current].delay > 0) {
        cycle_starts[cycle_starts_current].delay--;
      }
      else if (cycle_starts[cycle_starts_current].attempts > 0) {
        cycle_starts[cycle_starts_current].attempts--;
      }
    }

    if (cycle_starts[cycle_starts_current].addr &&
        cycle_starts[cycle_starts_current].delay == 0 &&
        cycle_starts[cycle_starts_current].attempts == 0)
      remove_cycle_start(RTIMER_NOW(), cycle_starts[cycle_starts_current].addr);
    else
      cycle_starts_current = (cycle_starts_current + 1) % cycle_starts_size;

    if (RTIMER_CLOCK_LT(cycle_start + CYCLE_TIME, RTIMER_NOW()))
    {
      cycle_start += CYCLE_TIME;
    }

    if (cycle_starts_current == 0)
    {
      cycle_start += CYCLE_TIME;
    }

    packet_seen = 0;

    for(count = 0; count < CCA_COUNT_MAX; ++count) {
      t0 = RTIMER_NOW();
//      PRINTF("contikimac: count %u t0 %u\n", count, t0);
      if(we_are_sending == 0 && we_are_receiving_burst == 0) {
        powercycle_turn_radio_on();
        /* Check if a packet is seen in the air. If so, we keep the
             radio on for a while (LISTEN_TIME_AFTER_PACKET_DETECTED) to
             be able to receive the packet. We also continuously check
             the radio medium to make sure that we were not woken up by a
             false positive: a spurious radio interference that was not
             caused by an incoming packet. */
        if(NETSTACK_RADIO.channel_clear() == 0) {
          packet_seen = 1;
          break;
        }
        powercycle_turn_radio_off();
      }
      schedule_powercycle_fixed(t, RTIMER_NOW() + CCA_SLEEP_TIME);
      PT_YIELD(&pt);
    }

    if(packet_seen) {
      static rtimer_clock_t start;
      static uint8_t silence_periods, periods;
      start = RTIMER_NOW();

      periods = silence_periods = 0;
      while(we_are_sending == 0 && radio_is_on &&
            RTIMER_CLOCK_LT(RTIMER_NOW(),
                            (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {

        /* Check for a number of consecutive periods of
             non-activity. If we see two such periods, we turn the
             radio off. Also, if a packet has been successfully
             received (as indicated by the
             NETSTACK_RADIO.pending_packet() function), we stop
             snooping. */
#if !RDC_CONF_HARDWARE_CSMA
       /* A cca cycle will disrupt rx on some radios, e.g. mc1322x, rf230 */
       /*TODO: Modify those drivers to just return the internal RSSI when already in rx mode */
        if(NETSTACK_RADIO.channel_clear()) {
          ++silence_periods;
        } else {
          silence_periods = 0;
        }
#endif

        ++periods;

        if(NETSTACK_RADIO.receiving_packet()) {
          silence_periods = 0;
        }
        if(silence_periods > MAX_SILENCE_PERIODS) {
          powercycle_turn_radio_off();
          break;
        }
        if(WITH_FAST_SLEEP &&
            periods > MAX_NONACTIVITY_PERIODS &&
            !(NETSTACK_RADIO.receiving_packet() ||
              NETSTACK_RADIO.pending_packet())) {
          powercycle_turn_radio_off();
          break;
        }
        if(NETSTACK_RADIO.pending_packet()) {
          break;
        }

        schedule_powercycle(t, CCA_CHECK_TIME + CCA_SLEEP_TIME);
        PT_YIELD(&pt);
      }
      if(radio_is_on) {
        if(!(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet()) ||
             !RTIMER_CLOCK_LT(RTIMER_NOW(),
                 (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {
          powercycle_turn_radio_off();
        }
      }
    }

    //printf("now: %u, cs: %u, idx: %u\n",(unsigned)(RTIMER_NOW()),(unsigned)cycle_starts[cycle_starts_index],(unsigned)cycle_starts_index);
    //if(RTIMER_CLOCK_LT(RTIMER_NOW() - (base_cycle_start + cycle_starts[cycle_starts_current].offset), CYCLE_TIME - CHECK_TIME * 4)) {
    if(RTIMER_CLOCK_LT(RTIMER_NOW() + CHECK_TIME * 4, cycle_start + cycle_starts[cycle_starts_current].offset)) {
      /* Schedule the next powercycle interrupt, or sleep the mcu
	 until then.  Sleeping will not exit from this interrupt, so
	 ensure an occasional wake cycle or foreground processing will
	 be blocked until a packet is detected */
#if RDC_CONF_MCU_SLEEP
      static uint8_t sleepcycle;
      if((sleepcycle++ < 16) && !we_are_sending && !radio_is_on) {
        rtimer_arch_sleep(CYCLE_TIME - (RTIMER_NOW() - cycle_start));
      } else {
        sleepcycle = 0;
        schedule_powercycle_fixed(t, CYCLE_TIME + cycle_starts[cycle_starts_index]);
        PT_YIELD(&pt);
      }
#else
      next_cycle_start = cycle_start + cycle_starts[cycle_starts_current].offset;
      schedule_powercycle_fixed(t, cycle_start + cycle_starts[cycle_starts_current].offset);
      //schedule_powercycle_fixed(t, RTIMER_NOW() + CYCLE_TIME);
      PT_YIELD(&pt);
#endif
    }

    if (cycle_starts[cycle_starts_current].addr == 0)
    {
      if (cycle_starts[cycle_starts_current].attempts)
        cc2420_set_out_of_phase_ack(0b01);
      else
        cc2420_set_out_of_phase_ack(0b00);
    }
    else
      cc2420_set_out_of_phase_ack(0b10);
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static int
broadcast_rate_drop(void)
{
#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
  if(!timer_expired(&broadcast_rate_timer)) {
    broadcast_rate_counter++;
    if(broadcast_rate_counter < CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT) {
      return 0;
    } else {
      return 1;
    }
  } else {
    timer_set(&broadcast_rate_timer, CLOCK_SECOND);
    broadcast_rate_counter = 0;
    return 0;
  }
#else /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
  return 0;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
}

void
contikimac_set_phase_for_routing(rimeaddr_t * addr, uint8_t rank); // forward

/*---------------------------------------------------------------------------*/
static int
send_packet(mac_callback_t mac_callback, void *mac_callback_ptr,
	    struct rdc_buf_list *buf_list,
            int is_receiver_awake)
{
  rtimer_clock_t t0;
  rtimer_clock_t encounter_time = 0;
  int strobes;
  uint8_t got_strobe_ack = 0;
  int hdrlen, len;
  uint8_t is_broadcast = 0;
  uint8_t is_reliable = 0;
  uint8_t is_known_receiver = 0;
  uint8_t collisions;
  int transmit_len;
  int ret;
  uint8_t contikimac_was_on;
  uint8_t seqno;
  uint8_t ack_reverse_phase = 0;
  uint8_t ack_no_phase = 0;
#if WITH_CONTIKIMAC_HEADER
  struct hdr *chdr;
#endif /* WITH_CONTIKIMAC_HEADER */

  /* Exit if RDC and radio were explicitly turned off */
   if(!contikimac_is_on && !contikimac_keep_radio_on) {
    PRINTF("contikimac: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }
 
  if(packetbuf_totlen() == 0) {
    PRINTF("contikimac: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

#if !NETSTACK_CONF_BRIDGE_MODE
  /* If NETSTACK_CONF_BRIDGE_MODE is set, assume PACKETBUF_ADDR_SENDER is already set. */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
#endif
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    is_broadcast = 1;
    PRINTDEBUG("contikimac: send broadcast\n");

    if(broadcast_rate_drop()) {
      return MAC_TX_COLLISION;
    }
  } else {
#if UIP_CONF_IPV6
    PRINTDEBUG("contikimac: send unicast to %02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[2],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[3],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[4],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[5],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[6],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7]);
#else /* UIP_CONF_IPV6 */
    PRINTDEBUG("contikimac: send unicast to %u.%u\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
#endif /* UIP_CONF_IPV6 */
  }
  is_reliable = packetbuf_attr(PACKETBUF_ATTR_RELIABLE) ||
    packetbuf_attr(PACKETBUF_ATTR_ERELIABLE);

  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

#if WITH_CONTIKIMAC_HEADER
  hdrlen = packetbuf_totlen();
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for contikimac header */
    PRINTF("contikimac: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
  chdr = packetbuf_hdrptr();
  chdr->id = CONTIKIMAC_ID;
  chdr->len = hdrlen;
  
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("contikimac: send failed, too large header\n");
    packetbuf_hdr_remove(sizeof(struct hdr));
    return MAC_TX_ERR_FATAL;
  }
  hdrlen += sizeof(struct hdr);
#else
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("contikimac: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
#endif

  /* Make sure that the packet is longer or equal to the shortest
     packet length. */
  transmit_len = packetbuf_totlen();
  if(transmit_len < SHORTEST_PACKET_SIZE) {
    /* Pad with zeroes */
    uint8_t *ptr;
    ptr = packetbuf_dataptr();
    memset(ptr + packetbuf_datalen(), 0, SHORTEST_PACKET_SIZE - packetbuf_totlen());

    PRINTF("contikimac: shorter than shortest (%d)\n", packetbuf_totlen());
    transmit_len = SHORTEST_PACKET_SIZE;
  }


  packetbuf_compact();

  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), transmit_len);

  /* Remove the MAC-layer header since it will be recreated next time around. */
  packetbuf_hdr_remove(hdrlen);

  #define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
  uint8_t req_resp_is_resp = (UIP_IP_BUF->tcflow & 0b00001100) == 0b00001100;

  if(!is_broadcast && !is_receiver_awake && !req_resp_is_resp) {
#if WITH_PHASE_OPTIMIZATION
    ret = phase_wait(&phase_list, packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     GUARD_TIME,
                     mac_callback, mac_callback_ptr, buf_list);
    if(ret == PHASE_DEFERRED) {
      PRINTF("contikimac: send failed, MAC_TX_DEFERRED\n");
      return MAC_TX_DEFERRED;
    }
    if(ret != PHASE_UNKNOWN) {
      is_known_receiver = 1;
    }
#endif /* WITH_PHASE_OPTIMIZATION */ 
  }
  


  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;

  /* If we have a pending packet in the radio, we should not send now,
     because we will trash the received packet. Instead, we signal
     that we have a collision, which lets the packet be received. This
     packet will be retransmitted later by the MAC protocol
     instead. */
  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    we_are_sending = 0;
    PRINTF("contikimac: collision receiving %d, pending %d\n",
           NETSTACK_RADIO.receiving_packet(), NETSTACK_RADIO.pending_packet());
    return MAC_TX_COLLISION;
  }
  
  /* Switch off the radio to ensure that we didn't start sending while
     the radio was doing a channel check. */
  off();


  strobes = 0;

  /* Send a train of strobes until the receiver answers with an ACK. */
  collisions = 0;

  got_strobe_ack = 0;

  /* Set contikimac_is_on to one to allow the on() and off() functions
     to control the radio. We restore the old value of
     contikimac_is_on when we are done. */
  contikimac_was_on = contikimac_is_on;
  contikimac_is_on = 1;

#if !RDC_CONF_HARDWARE_CSMA
    /* Check if there are any transmissions by others. */
    /* TODO: why does this give collisions before sending with the mc1322x? */
  if(is_receiver_awake == 0) {
    int i;
    for(i = 0; i < CCA_COUNT_MAX_TX; ++i) {
      t0 = RTIMER_NOW();
      on();
#if CCA_CHECK_TIME > 0
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)) { }
#endif
      if(NETSTACK_RADIO.channel_clear() == 0) {
        collisions++;
        off();
        break;
      }
      off();
      t0 = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_SLEEP_TIME)) { }
    }
  }

  if(collisions > 0) {
    we_are_sending = 0;
    off();
    PRINTF("contikimac: collisions before sending\n");
    contikimac_is_on = contikimac_was_on;
    return MAC_TX_COLLISION;
  }
#endif /* RDC_CONF_HARDWARE_CSMA */

#if !RDC_CONF_HARDWARE_ACK
  if(!is_broadcast) {
    /* Turn radio on to receive expected unicast ack.  Not necessary
       with hardware ack detection, and may trigger an unnecessary cca
       or rx cycle */
     on();
  }
#endif

  watchdog_periodic();
  t0 = RTIMER_NOW();
  seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  for(strobes = 0, collisions = 0;
      got_strobe_ack == 0 && collisions == 0 &&
      RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + STROBE_TIME); strobes++) {

    watchdog_periodic();

/* ***************************************************************************************************************************************
 * Pietro: the following if statement stops the transmission after t0 + MAX_PHASE_STROBE_TIME, when the receiver is expected to be gone
 * back asleep. This could be a problem with SynchroMAC in case the receiver has shifted the phase. The phase locking takes care of this.
 * If the statement is disabled, packets are repeated for the entire cycle time, exhausting the channel capacity especially if the channel
 * check rate is low.
 *****************************************************************************************************************************************/
    //if(!is_broadcast && (is_receiver_awake || is_known_receiver) &&
    //   !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MAX_PHASE_STROBE_TIME)) {
    //  PRINTF("miss to %d\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]);
    //  break;
    //}

    len = 0;

    {
      rtimer_clock_t wt;
      rtimer_clock_t txtime;
      int ret;

      txtime = RTIMER_NOW();
      ret = NETSTACK_RADIO.transmit(transmit_len);

#if RDC_CONF_HARDWARE_ACK
     /* For radios that block in the transmit routine and detect the
	ACK in hardware */
      if(ret == RADIO_TX_OK) {
        if(!is_broadcast) {
          got_strobe_ack = 1;
          encounter_time = txtime;
          break;
        }
      } else if (ret == RADIO_TX_NOACK) {
      } else if (ret == RADIO_TX_COLLISION) {
          PRINTF("contikimac: collisions while sending\n");
          collisions++;
      }
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }
#else /* RDC_CONF_HARDWARE_ACK */
     /* Wait for the ACK packet */
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }

      if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
                           NETSTACK_RADIO.pending_packet() ||
                           NETSTACK_RADIO.channel_clear() == 0)) {
        uint8_t ackbuf[ACK_LEN];
        wt = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + AFTER_ACK_DETECTECT_WAIT_TIME)) { }

        len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
        if(len == ACK_LEN && seqno == ackbuf[ACK_LEN - 1]) {
          got_strobe_ack = 1;
          encounter_time = txtime;
          ack_reverse_phase = !!(ackbuf[0] & (1<<7)); // ACK_OUT_OF_PHASE = (1<<7)
          ack_no_phase = !!(ackbuf[0] & (1<<5));      // using ACK_REQUEST = (1<<5)
          break;
        } else {
          PRINTF("contikimac: collisions while sending\n");
          collisions++;
        }
      }
#endif /* RDC_CONF_HARDWARE_ACK */
    }
  }

  off();

  PRINTF("contikimac: send (strobes=%u, len=%u, %s, %s), done\n", strobes,
         packetbuf_totlen(),
         got_strobe_ack ? "ack" : "no ack",
         collisions ? "collision" : "no collision");

#if CONTIKIMAC_CONF_COMPOWER
  /* Accumulate the power consumption for the packet transmission. */
  compower_accumulate(&current_packet);

  /* Convert the accumulated power consumption for the transmitted
     packet to packet attributes so that the higher levels can keep
     track of the amount of energy spent on transmitting the
     packet. */
  compower_attrconv(&current_packet);

  /* Clear the accumulated power consumption so that it is ready for
     the next packet. */
  compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

  contikimac_is_on = contikimac_was_on;

  /* Determine the return value that we will return from the
     function. We must pass this value to the phase module before we
     return from the function.  */
  if(collisions > 0) {
    ret = MAC_TX_COLLISION;
  } else if(!is_broadcast && !got_strobe_ack) {
    ret = MAC_TX_NOACK;
  } else {
    ret = MAC_TX_OK;
  }

#if WITH_PHASE_OPTIMIZATION
  if(is_known_receiver && got_strobe_ack) {
    PRINTF("no miss %d wake-ups %d\n",
	   packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
           strobes);
  }

  if(!is_broadcast) {
    if(collisions == 0 && is_receiver_awake == 0 && got_strobe_ack && !ack_no_phase) {

#if BOTH_RAWMAC_LADDERS
      unsigned int rank = 2; // all nodes connected to root node have rank 2
      rpl_dag_t *dag;
      rimeaddr_t rpl_parent_macaddr;

      dag = rpl_get_any_dag();
      if (dag && dag->preferred_parent) {
        rank = dag->instance->mc.distance_to_sink;
        uip_ds6_get_addr_iid(&(dag->preferred_parent->addr),(uip_lladdr_t *)&rpl_parent_macaddr);
        if(!rimeaddr_cmp(&rpl_parent_macaddr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER))) {
          rank += 2;
        }
      }

      if (ack_reverse_phase && rank)
      {
        rtimer_clock_t other_time = encounter_time;
        encounter_time += 2 * PHASE_OFFSET * (rank - 1);
        phase_update2(&phase_list, packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
          encounter_time, &other_time, ret);
        //printf("phase update out_of_phase rank %u\n",(unsigned)rank);
      }
      if (!ack_reverse_phase && rank)
      {
        rtimer_clock_t other_time = encounter_time - (2 * PHASE_OFFSET * (rank - 1));
        phase_update2(&phase_list, packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
          encounter_time, &other_time, ret);
        //printf("phase update in_phase rank %u\n",(unsigned)rank);
      }
#else
      phase_update(&phase_list, packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                   encounter_time, ret);
#endif

      // Pietro: if the receiver is the RPL preferred parent of this node, shift the wake-up phase
      {
    	  rimeaddr_t rpl_parent_macaddr;
    	  rpl_dag_t *dag;

    	  dag = rpl_get_any_dag();
        if (dag && dag->preferred_parent) {
          uip_ds6_get_addr_iid(&(dag->preferred_parent->addr),(uip_lladdr_t *)&rpl_parent_macaddr);
          if(rimeaddr_cmp(&rpl_parent_macaddr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER))) {
            contikimac_set_phase_for_routing(&rpl_parent_macaddr,dag->instance->mc.distance_to_sink);
          }
        }
      }
    }
  }
#endif /* WITH_PHASE_OPTIMIZATION */
  we_are_sending = 0;

  return ret;
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{
  int ret = send_packet(sent, ptr, NULL, 0);
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  struct rdc_buf_list *curr = buf_list;
  struct rdc_buf_list *next;
  int ret;
  int is_receiver_awake;
  
  if(curr == NULL) {
    return;
  }
  /* Do not send during reception of a burst */
  if(we_are_receiving_burst) {
    /* Prepare the packetbuf for callback */
    queuebuf_to_packetbuf(curr->buf);
    /* Return COLLISION so the MAC may try again later */
    mac_call_sent_callback(sent, ptr, MAC_TX_COLLISION, 1);
    return;
  }
  /* The receiver needs to be awoken before we send */
  is_receiver_awake = 0;
  do { /* A loop sending a burst of packets from buf_list */
    next = list_item_next(curr);

    /* Prepare the packetbuf */
    queuebuf_to_packetbuf(curr->buf);
    if(next != NULL) {
      packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 1);
    }

    /* Send the current packet */
    ret = send_packet(sent, ptr, curr, is_receiver_awake);
    if(ret != MAC_TX_DEFERRED) {
      mac_call_sent_callback(sent, ptr, ret, 1);
    }

    if(ret == MAC_TX_OK) {
      if(next != NULL) {
        /* We're in a burst, no need to wake the receiver up again */
        is_receiver_awake = 1;
        curr = next;
      }
    } else {
      /* The transmission failed, we stop the burst */
      next = NULL;
    }
  } while(next != NULL);
}
/*---------------------------------------------------------------------------*/
/* Timer callback triggered when receiving a burst, after having
   waited for a next packet for a too long time. Turns the radio off
   and leaves burst reception mode */
static void
recv_burst_off(void *ptr)
{
  off();
  we_are_receiving_burst = 0;
}
/*---------------------------------------------------------------------------*/
/* Function added by RMonica
 *
 * save cycle time information (in "newtime") for neighbor "addr"
 * simply delegate to the phase system (see phase.h)
 */
void contikimac_cycle_time_update(const rimeaddr_t * addr,rtimer_cycle_time_t newtime)
{
#if WITH_PHASE_OPTIMIZATION
  if (newtime < APPROX_RADIO_ALWAYS_ON_CYCLE_TIME) {
    newtime = 0;
  }

  //PRINTF("contikimac_cycle_time_update: %d source: %u\n",((int)newtime),(int)(addr->u8[7]));
  cycle_time_update(&phase_list, addr, newtime);
#endif
}
/*---------------------------------------------------------------------------*/
/* Function added by RMonica
 * returns the cycle time of this node, to be broadcasted by RPL DIOs
 */
rtimer_cycle_time_t contikimac_get_cycle_time_for_routing()
{
  if (contikimac_keep_radio_on)
    return 0; // 0 means "radio always on"
  return CYCLE_TIME;
}
/* Function added by RMonica
 * get Minimum Forwarding Time: the minimum time needed to forward a packet
 */
/*---------------------------------------------------------------------------*/
rtimer_cycle_time_t contikimac_get_MFT_for_routing()
{
  return 2 * GUARD_TIME;
}
/*---------------------------------------------------------------------------*/
/* Function added by RMonica
 * get average communication delay (due to duty cycle) towards node "toNode"
 */
rtimer_cycle_time_t contikimac_get_average_delay_for_routing(const rimeaddr_t * toNode)
{
#if WITH_PHASE_OPTIMIZATION
  return phase_get_average_delay(&phase_list, toNode,( 2 * GUARD_TIME), cycle_start);
#else
  return (CYCLE_TIME >> 1) + (2 * GUARD_TIME);
#endif
}
/*---------------------------------------------------------------------------*/
/*
 * Function added by Pietro Gonizzi
 * Description: shift the wake-up phase of the node to meet routing QoS
 */
void
contikimac_set_phase_for_routing(rimeaddr_t * addr,uint8_t rank)
{
  cycle_starts_mutex = 1;
	rtimer_clock_t cycle_offset = phase_get_neighbor_phase(&phase_list, addr);

  if(cycle_offset != 0) {
		//printf("The phase of %u is known\n", addr->u8[7]);
		// TODO: shift only if not done already
		// solution 1: store the parent node address in a local variable
		// solution 2: check the phase offset to the parent node address and adjust if needed (use this)
		//PRINTF("contikimac: cycle_start %u, cycle_offset %u, CYCLE_TIME %u\n", cycle_start, cycle_offset, CYCLE_TIME);
		//PRINTF("contikimac: (cycle_offset - cycle_start) mod CYCLE_TIME %u\n", (cycle_offset - cycle_start) % CYCLE_TIME);
		//PRINTF("contikimac: (cycle_start - cycle_offset) mod CYCLE_TIME %u\n", (cycle_start - cycle_offset) % CYCLE_TIME);
#if OPTIMIZE_TO_SINK
    if((cycle_offset - cycle_starts[first_cycle_start].offset + CYCLE_TIME) % CYCLE_TIME < (PHASE_OFFSET - GUARD_TIME/2) ||
       (cycle_offset - cycle_starts[first_cycle_start].offset + CYCLE_TIME) % CYCLE_TIME > (PHASE_OFFSET + GUARD_TIME/2) ||
       rank != cycle_prev_rank) {
#else
    if((cycle_starts[first_cycle_start].offset + CYCLE_TIME - cycle_offset) % CYCLE_TIME < (PHASE_OFFSET - GUARD_TIME/2) ||
       (cycle_starts[first_cycle_start].offset + CYCLE_TIME - cycle_offset) % CYCLE_TIME > (PHASE_OFFSET + GUARD_TIME/2) ||
       rank != cycle_prev_rank) {
#endif
			//printf("(cycle_offset - cycle_start) mod CYCLE_TIME = %u, 2*GUARD_TIME = %u, CCA_SLEEP_TIME %u\n", (cycle_offset - cycle_start) % CYCLE_TIME, 2*GUARD_TIME, CCA_SLEEP_TIME);
//			printf("contikimac: Shifting phase from %u to %u\n", cycle_start, cycle_offset - 2*GUARD_TIME);
      cycle_prev_rank = rank;
      rtimer_clock_t cycle_phase_offset = (2 * PHASE_OFFSET * rank) % CYCLE_TIME;
			powercycle_turn_radio_off();
      rtimer_clock_t second_offset;
#if OPTIMIZE_TO_SINK
      cycle_starts[first_cycle_start].offset = (CYCLE_TIME + cycle_offset - PHASE_OFFSET) % CYCLE_TIME;
      second_offset = (cycle_starts[first_cycle_start].offset + PHASE_OFFSET) % CYCLE_TIME;
#else
      cycle_starts[first_cycle_start].offset = (cycle_offset + PHASE_OFFSET) % CYCLE_TIME;
      second_offset =
        (cycle_starts[first_cycle_start].offset + CYCLE_TIME - cycle_phase_offset) % CYCLE_TIME;
#endif
      //printf("rank: %u, cycle_phase_offset: %u, cycle start 0: %u, cycle start 1: %u, cycle_start_index: %u\n",
      //       (unsigned)rank,(unsigned)cycle_phase_offset,(unsigned)(cycle_starts[0]),(unsigned)(cycle_starts[1]),
      //       (unsigned)cycle_starts_index);
#if BOTH_RAWMAC_LADDERS
      rtimer_clock_t now = RTIMER_NOW();
      if (second_cycle_start == (uint8_t)(-1))
      {
        insert_new_cycle_start(second_offset, 0, 1, 0, now);
      }
      else
      {
        cycle_starts[second_cycle_start].offset = second_offset;
        reorder_cycle_starts(now);
      }
#endif
			//powercycle_turn_radio_on();
		}
  }

  cycle_starts_mutex = 0;

// TODO: check for any ongoing transmissions/receptions before switching off brutally
}

static void
input_packet(void)
{
  static struct ctimer ct;
  if(!we_are_receiving_burst) {
    off();
  }

  /*  PRINTF("cycle_start 0x%02x 0x%02x\n", cycle_start, cycle_start % CYCLE_TIME);*/
  
  if(packetbuf_totlen() > 0 && NETSTACK_FRAMER.parse() >= 0) {

#if WITH_CONTIKIMAC_HEADER
    struct hdr *chdr;
    chdr = packetbuf_dataptr();
    if(chdr->id != CONTIKIMAC_ID) {
      PRINTF("contikimac: failed to parse hdr (%u)\n", packetbuf_totlen());
      return;
    }
    packetbuf_hdrreduce(sizeof(struct hdr));
    packetbuf_set_datalen(chdr->len);
#endif /* WITH_CONTIKIMAC_HEADER */

    if(packetbuf_datalen() > 0 &&
       packetbuf_totlen() > 0 &&
       (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_node_addr) ||
        rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &rimeaddr_null))) {
      /* This is a regular packet that is destined to us or to the
         broadcast address. */

      /* If FRAME_PENDING is set, we are receiving a packets in a burst */
      we_are_receiving_burst = packetbuf_attr(PACKETBUF_ATTR_PENDING);
      if(we_are_receiving_burst) {
        on();
        /* Set a timer to turn the radio off in case we do not receive
	   a next packet */
        ctimer_set(&ct, INTER_PACKET_DEADLINE, recv_burst_off, NULL);
      } else {
        off();
        ctimer_stop(&ct);
      }

      /* Check for duplicate packet by comparing the sequence number
         of the incoming packet with the last few ones we saw. */
      {
        int i;
        for(i = 0; i < MAX_SEQNOS; ++i) {
          if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
             rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
                          &received_seqnos[i].sender)) {
            /* Drop the packet. */
            /*        PRINTF("Drop duplicate ContikiMAC layer packet\n");*/
            return;
          }
        }
        for(i = MAX_SEQNOS - 1; i > 0; --i) {
          memcpy(&received_seqnos[i], &received_seqnos[i - 1],
                 sizeof(struct seqno));
        }
        received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
        rimeaddr_copy(&received_seqnos[0].sender,
                      packetbuf_addr(PACKETBUF_ADDR_SENDER));
      }

#if CONTIKIMAC_CONF_COMPOWER
      /* Accumulate the power consumption for the packet reception. */
      compower_accumulate(&current_packet);
      /* Convert the accumulated power consumption for the received
         packet to packet attributes so that the higher levels can
         keep track of the amount of energy spent on receiving the
         packet. */
      compower_attrconv(&current_packet);

      /* Clear the accumulated power consumption so that it is ready
         for the next packet. */
      compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

      PRINTDEBUG("contikimac: data (%u)\n", packetbuf_datalen());
      NETSTACK_MAC.input();
      return;
    } else {
      PRINTDEBUG("contikimac: data not for us\n");
    }
  } else {
    PRINTF("contikimac: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  radio_is_on = 0;
  cycle_starts_mutex = 0;
  PT_INIT(&pt);

  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
             (void (*)(struct rtimer *, void *))powercycle, NULL);

  contikimac_is_on = 1;

#if WITH_PHASE_OPTIMIZATION
  phase_init(&phase_list);
#endif /* WITH_PHASE_OPTIMIZATION */
  PRINTF("contikimac: init done\n");

}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  PRINTF("contikimac: turn on\n");
  if(contikimac_is_on == 0) {
    contikimac_is_on = 1;
    contikimac_keep_radio_on = 0;
    rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
               (void (*)(struct rtimer *, void *))powercycle, NULL);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  PRINTF("contikimac: turn off\n");
  contikimac_is_on = 0;
  contikimac_keep_radio_on = keep_radio_on;
  if(keep_radio_on) {
    radio_is_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    radio_is_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
duty_cycle(void)
{
  return (1ul * CLOCK_SECOND * CYCLE_TIME) / RTIMER_ARCH_SECOND;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver synchro_contikimac_driver = {
  "SynchroContikiMAC",
  init,
  qsend_packet,
  qsend_list,
  input_packet,
  turn_on,
  turn_off,
  duty_cycle,
};
/*---------------------------------------------------------------------------*/
uint16_t
contikimac_debug_print(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/

/*void contikimac_get_cycle_start(void)
{
	PRINTF("cycle_start %u\n", cycle_start);
	PRINTF("contikimac: CYCLE_TIME %u\n", CYCLE_TIME);
	PRINTF("contikimac: RTIMER_ARCH_SECOND %u\n", RTIMER_ARCH_SECOND);
}*/
