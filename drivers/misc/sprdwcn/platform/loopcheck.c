#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "wcn_glb.h"
#include "wcn_misc.h"
#include "wcn_procfs.h"

#define LOOPCHECK_TIMER_INTERVAL      5
#define WCN_LOOPCHECK_INIT	1
#define WCN_LOOPCHECK_OPEN	2
#define WCN_LOOPCHECK_FAIL	3

struct wcn_loopcheck {
	unsigned long status;
	struct completion completion;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
};

static struct wcn_loopcheck loopcheck;

static int loopcheck_send(char *buf, unsigned int len)
{
	unsigned char *send_buf = NULL;
	struct mbuf_t *head = NULL;
	struct mbuf_t *tail = NULL;
	struct mchn_ops_t *mchn_ops = &mdbg_proc_ops[MDBG_AT_TX_OPS];
	int ret = 0, num = 1;

	WCN_INFO("tx:%s\n", buf);
	if (unlikely(!marlin_get_module_status())) {
		WCN_ERR("WCN module have not open\n");
		return -EIO;
	}

	send_buf = kzalloc(len + PUB_HEAD_RSV + 1, GFP_KERNEL);
	if (!send_buf)
		return -ENOMEM;
	memcpy(send_buf + PUB_HEAD_RSV, buf, len);

	if (!sprdwcn_bus_list_alloc(mchn_ops->channel, &head, &tail, &num)) {
		head->buf = send_buf;
		head->len = len;
		head->next = NULL;
#ifdef CONFIG_SDIOHAL
		ret = sprdwcn_bus_push_list_direct(mchn_ops->channel,
						   head, tail, num);

		if (mchn_ops->pop_link)
			mchn_ops->pop_link(mchn_ops->channel,
					   head, tail, num);
		else
			sprdwcn_bus_list_free(mchn_ops->channel,
					      head, tail, num);
#else
		ret = sprdwcn_bus_push_list(mchn_ops->channel,
					    head, tail, num);
#endif
		if (ret != 0)
			WCN_ERR("loopcheck send fail!\n");
	} else {
		WCN_ERR("%s alloc buf fail!\n", __func__);
		kfree(send_buf);
		return -ENOMEM;
	}
	return ret;
}

static void loopcheck_work_queue(struct work_struct *work)
{
	int ret;
	char a[64];
	unsigned long timeleft;
	unsigned long long sprdwcn_rx_cnt_a = 0, sprdwcn_rx_cnt_b = 0;
	unsigned long long loopcheck_tx_ns, marlin_boot_t;

	loopcheck_tx_ns = local_clock();
	marlin_boot_t = marlin_bootup_time_get();
	MARLIN_64B_NS_TO_32B_MS(loopcheck_tx_ns);
	MARLIN_64B_NS_TO_32B_MS(marlin_boot_t);
	snprintf(a, (size_t)sizeof(a), "at+loopcheck=%llu,%llu\r\n",
		 loopcheck_tx_ns, marlin_boot_t);

	if (!test_bit(WCN_LOOPCHECK_OPEN, &loopcheck.status))
		return;

	sprdwcn_rx_cnt_a = sprdwcn_bus_get_rx_total_cnt();
	usleep_range(4000, 6000);
	sprdwcn_rx_cnt_b = sprdwcn_bus_get_rx_total_cnt();

	if (sprdwcn_rx_cnt_a == sprdwcn_rx_cnt_b) {
		loopcheck_send(a, strlen(a));
		timeleft = wait_for_completion_timeout(&loopcheck.completion,
						       (4 * HZ));
		if (!test_bit(WCN_LOOPCHECK_OPEN, &loopcheck.status))
			return;
		if (!timeleft) {
			set_bit(WCN_LOOPCHECK_FAIL, &loopcheck.status);
			WCN_ERR("didn't get loopcheck ack, printk=%d\n",
				console_loglevel);
			mdbg_assert_interface("WCN loopcheck erro!");
			clear_bit(WCN_LOOPCHECK_FAIL, &loopcheck.status);
			return;
		}
	}

	ret = queue_delayed_work(loopcheck.workqueue, &loopcheck.work,
				 LOOPCHECK_TIMER_INTERVAL * HZ);
}

void start_loopcheck(void)
{
	if (!test_bit(WCN_LOOPCHECK_INIT, &loopcheck.status) ||
	    test_and_set_bit(WCN_LOOPCHECK_OPEN, &loopcheck.status))
		return;
	WCN_INFO("%s\n", __func__);
	reinit_completion(&loopcheck.completion);
	queue_delayed_work(loopcheck.workqueue, &loopcheck.work, HZ);
}

void stop_loopcheck(void)
{
	if (!test_bit(WCN_LOOPCHECK_INIT, &loopcheck.status) ||
	    !test_and_clear_bit(WCN_LOOPCHECK_OPEN, &loopcheck.status) ||
	    test_bit(WCN_LOOPCHECK_FAIL, &loopcheck.status))
		return;
	WCN_INFO("%s\n", __func__);
	complete_all(&loopcheck.completion);
	cancel_delayed_work_sync(&loopcheck.work);
}

void complete_kernel_loopcheck(void)
{
	complete(&loopcheck.completion);
}

int loopcheck_init(void)
{
	loopcheck.status = 0;
	init_completion(&loopcheck.completion);
	loopcheck.workqueue =
		create_singlethread_workqueue("WCN_LOOPCHECK_QUEUE");
	if (!loopcheck.workqueue) {
		WCN_ERR("WCN_LOOPCHECK_QUEUE create failed");
		return -ENOMEM;
	}
	set_bit(WCN_LOOPCHECK_INIT, &loopcheck.status);
	INIT_DELAYED_WORK(&loopcheck.work, loopcheck_work_queue);

	return 0;
}

int loopcheck_deinit(void)
{
	stop_loopcheck();
	destroy_workqueue(loopcheck.workqueue);
	loopcheck.status = 0;

	return 0;
}
