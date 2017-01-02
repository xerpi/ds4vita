#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/bt.h>
#include <psp2kern/ctrl.h>
#include <psp2/touch.h>
#include <taihen.h>
#include "log.h"

extern int ksceKernelPowerTick(int);

#define DS4_VID 0x054C
#define DS4_PID 0x05C4

#define DS4_TOUCHPAD_W 1920
#define DS4_TOUCHPAD_H 940
#define DS4_ANALOG_THRESHOLD 5

#define VITA_FRONT_TOUCHSCREEN_W 1920
#define VITA_FRONT_TOUCHSCREEN_H 1080

#define abs(x) (((x) < 0) ? -(x) : (x))

struct ds4_input_report {
	unsigned char report_id;
	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned char dpad     : 4;
	unsigned char square   : 1;
	unsigned char cross    : 1;
	unsigned char circle   : 1;
	unsigned char triangle : 1;

	unsigned char l1      : 1;
	unsigned char r1      : 1;
	unsigned char l2      : 1;
	unsigned char r2      : 1;
	unsigned char share   : 1;
	unsigned char options : 1;
	unsigned char l3      : 1;
	unsigned char r3      : 1;

	unsigned char ps   : 1;
	unsigned char tpad : 1;
	unsigned char cnt1 : 6;

	unsigned char l_trigger;
	unsigned char r_trigger;

	unsigned char cnt2;
	unsigned char cnt3;

	unsigned char battery;

	signed short accel_x;
	signed short accel_y;
	signed short accel_z;

	union {
		signed short roll;
		signed short gyro_z;
	};
	union {
		signed short yaw;
		signed short gyro_y;
	};
	union {
		signed short pitch;
		signed short gyro_x;
	};

	unsigned char unk1[5];

	unsigned char battery_level : 4;
	unsigned char usb_plugged   : 1;
	unsigned char headphones    : 1;
	unsigned char microphone    : 1;
	unsigned char padding       : 1;

	unsigned char unk2[2];
	unsigned char trackpadpackets;
	unsigned char packetcnt;

	unsigned int finger1_id        : 7;
	unsigned int finger1_activelow : 1;
	unsigned int finger1_x         : 12;
	unsigned int finger1_y         : 12;

	unsigned int finger2_id        : 7;
	unsigned int finger2_activelow : 1;
	unsigned int finger2_x         : 12;
	unsigned int finger2_y         : 12;

} __attribute__((packed, aligned(32)));

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

static int ds4_connected = 0;
static unsigned int ds4_mac0 = 0;
static unsigned int ds4_mac1 = 0;

static struct ds4_input_report ds4_input;

static tai_hook_ref_t SceBt_sub_22999C8_ref;
static SceUID SceBt_sub_22999C8_hook_uid = -1;
static tai_hook_ref_t SceTouch_sceTouchPeek_ref;
static SceUID SceTouch_sceTouchPeek_hook_uid = -1;
static tai_hook_ref_t SceTouch_sceTouchPeekRegion_ref;
static SceUID SceTouch_sceTouchPeekRegion_hook_uid = -1;

static inline void ds4_input_reset(void)
{
	memset(&ds4_input, 0, sizeof(ds4_input));
}

static int is_ds4(const unsigned short vid_pid[2])
{
	return vid_pid[0] == DS4_VID && vid_pid[1] == DS4_PID;
}

static inline void *mempool_alloc(unsigned int size)
{
	return ksceKernelMemPoolAlloc(bt_mempool_uid, size);
}

static inline void mempool_free(void *ptr)
{
	ksceKernelMemPoolFree(bt_mempool_uid, ptr);
}

static int ds4_send_report(unsigned int mac0, unsigned int mac1, uint8_t flags, uint8_t report,
			    size_t len, const void *data)
{
	SceBtHidRequest *req;
	unsigned char *buf;

	req = mempool_alloc(sizeof(*req));
	if (!req) {
		LOG("Error allocatin BT HID Request\n");
		return -1;
	}

	if ((buf = mempool_alloc((len + 1) * sizeof(*buf))) == NULL) {
		LOG("Memory allocation error (mesg array)\n");
		return -1;
	}

	buf[0] = report;
	memcpy(buf + 1, data, len);

	memset(req, 0, sizeof(*req));
	req->type = 1; // 0xA2 -> type = 1
	req->buffer = buf;
	req->length = len + 1;
	req->next = req;

	TEST_CALL(ksceBtHidTransfer, mac0, mac1, req);

	mempool_free(buf);
	mempool_free(req);

	return 0;
}

static int ds4_send_0x11_report(unsigned int mac0, unsigned int mac1)
{
	unsigned char data[] = {
		0x80,
		0x0F,
		0x00,
		0x00,
		0x00,
		0x00, // Motor right
		0x00, // Motor left
		0xFF, // R
		0x00, // G
		0xFF, // B
		0xFF, // Blink on
		0x00, // Blink off
	};

	if (ds4_send_report(mac0, mac1, 0, 0x11, sizeof(data), data)) {
		LOG("Status request error\n");
		return -1;
	}

	return 0;
}

static void reset_input_emulation()
{
	ksceCtrlSetButtonEmulation(0, 0, 0, 0, 32);
	ksceCtrlSetAnalogEmulation(0, 0, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0);
}

static void set_input_emulation(struct ds4_input_report *ds4)
{
	unsigned int buttons = 0;
	int js_moved = 0;

	if (ds4->cross)
		buttons |= SCE_CTRL_CROSS;
	if (ds4->circle)
		buttons |= SCE_CTRL_CIRCLE;
	if (ds4->triangle)
		buttons |= SCE_CTRL_TRIANGLE;
	if (ds4->square)
		buttons |= SCE_CTRL_SQUARE;

	if (ds4->dpad == 0 || ds4->dpad == 1 || ds4->dpad == 7)
		buttons |= SCE_CTRL_UP;
	if (ds4->dpad == 1 || ds4->dpad == 2 || ds4->dpad == 3)
		buttons |= SCE_CTRL_RIGHT;
	if (ds4->dpad == 3 || ds4->dpad == 4 || ds4->dpad == 5)
		buttons |= SCE_CTRL_DOWN;
	if (ds4->dpad == 5 || ds4->dpad == 6 || ds4->dpad == 7)
		buttons |= SCE_CTRL_LEFT;

	if (ds4->l1 || ds4->l2)
		buttons |= (SCE_CTRL_L1 | SCE_CTRL_LTRIGGER);
	if (ds4->r1 || ds4->r2)
		buttons |= (SCE_CTRL_R1 | SCE_CTRL_RTRIGGER);

	if (ds4->share)
		buttons |= SCE_CTRL_SELECT;
	if (ds4->options)
		buttons |= SCE_CTRL_START;
	if (ds4->ps)
		buttons |= SCE_CTRL_INTERCEPTED;

	if ((abs(ds4->left_x - 128) > DS4_ANALOG_THRESHOLD) ||
	    (abs(ds4->left_y - 128) > DS4_ANALOG_THRESHOLD) ||
	    (abs(ds4->right_x - 128) > DS4_ANALOG_THRESHOLD) ||
	    (abs(ds4->right_y - 128) > DS4_ANALOG_THRESHOLD)) {
		js_moved = 1;
	}

	ksceCtrlSetButtonEmulation(0, 0, buttons, buttons, 32);

	ksceCtrlSetAnalogEmulation(0, 0, ds4->left_x, ds4->left_y,
		ds4->right_x, ds4->right_y, ds4->left_x, ds4->left_y,
		ds4->right_x, ds4->right_y, 32);

	if (buttons != 0 || js_moved)
		ksceKernelPowerTick(0);
}

static void patch_touchdata(SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs,
			    struct ds4_input_report *ds4)
{
	int i;
	SceTouchData *utouch_data = pData;

	for (i = 0; i < nBufs; i++) {
		SceTouchData ktouch_data;
		int num_reports = 0;

		ksceKernelMemcpyUserToKernel(&ktouch_data, (uintptr_t)utouch_data, sizeof(ktouch_data));

		if (!ds4->finger1_activelow) {
			ktouch_data.report[0].id = ds4->finger1_id;
			ktouch_data.report[0].x = (ds4->finger1_x * VITA_FRONT_TOUCHSCREEN_W) / DS4_TOUCHPAD_W;
			ktouch_data.report[0].y = (ds4->finger1_y * VITA_FRONT_TOUCHSCREEN_H) / DS4_TOUCHPAD_H;
			num_reports++;
		}

		if (!ds4->finger2_activelow) {
			ktouch_data.report[1].id = ds4->finger2_id;
			ktouch_data.report[1].x = (ds4->finger2_x * VITA_FRONT_TOUCHSCREEN_W) / DS4_TOUCHPAD_W;
			ktouch_data.report[1].y = (ds4->finger2_y * VITA_FRONT_TOUCHSCREEN_H) / DS4_TOUCHPAD_H;
			num_reports++;
		}

		if (num_reports > 0) {
			ksceKernelPowerTick(0);
			ktouch_data.reportNum = num_reports;
		}

		ksceKernelMemcpyKernelToUser((uintptr_t)utouch_data, &ktouch_data, sizeof(ktouch_data));

		utouch_data++;
	}
}

static int SceTouch_sceTouchPeek_hook_func(SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs)
{
	int ret;

	ret = TAI_CONTINUE(int, SceTouch_sceTouchPeek_ref, port, pData, nBufs);

	if (ret >= 0 && ds4_connected) {
		patch_touchdata(port, pData, nBufs, &ds4_input);
	}

	return ret;
}

static int SceTouch_sceTouchPeekRegion_hook_func(SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs, int region)
{
	int ret;

	ret = TAI_CONTINUE(int, SceTouch_sceTouchPeekRegion_ref, port, pData, nBufs, region);

	if (ret >= 0 && ds4_connected) {
		patch_touchdata(port, pData, nBufs, &ds4_input);
	}

	return ret;
}

static void enqueue_read_request(unsigned int mac0, unsigned int mac1,
				 SceBtHidRequest *request, unsigned char *buffer,
				 unsigned int length)
{
	memset(request, 0, sizeof(*request));
	memset(buffer, 0, length);

	request->type = 0;
	request->buffer = buffer;
	request->length = length;
	request->next = request;

	ksceBtHidTransfer(mac0, mac1, request);
}

static int SceBt_sub_22999C8_hook_func(void *dev_base_ptr, int r1)
{
	unsigned int flags = *(unsigned int *)(r1 + 4);

	if (dev_base_ptr && !(flags & 2)) {
		const void *dev_info = *(const void **)(dev_base_ptr + 0x14A4);
		const unsigned short *vid_pid = (const unsigned short *)(dev_info + 0x28);

		if (is_ds4(vid_pid)) {
			unsigned int *v8_ptr = (unsigned int *)(*(unsigned int *)dev_base_ptr + 8);

			/*
			 * We need to enable the following bits in order to make the Vita
			 * accept the new connection, otherwise it will refuse it.
			 */
			*v8_ptr |= 0x11000;
		}
	}

	return TAI_CONTINUE(int, SceBt_sub_22999C8_ref, dev_base_ptr, r1);
}

static int bt_cb_func(int notifyId, int notifyCount, int notifyArg, void *common)
{
	static SceBtHidRequest hid_request;
	static unsigned char recv_buff[0x100];

	while (1) {
		int ret;
		SceBtEvent hid_event;

		memset(&hid_event, 0, sizeof(hid_event));

		do {
			ret = ksceBtReadEvent(&hid_event, 1);
		} while (ret == SCE_BT_ERROR_CB_OVERFLOW);

		if (ret <= 0) {
			break;
		}

		LOG("->Event:");
		for (int i = 0; i < 0x10; i++)
			LOG(" %02X", hid_event.data[i]);
		LOG("\n");

		/*
		 * If we get an event with a MAC, and the MAC is different
		 * from the connected DS4, skip the event.
		 */
		if (ds4_connected) {
			if (hid_event.mac0 != ds4_mac0 || hid_event.mac1 != ds4_mac1)
				continue;
		}

		switch (hid_event.id) {
		case 0x01: { /* Inquiry result event */
			unsigned short vid_pid[2];
			ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);

			if (is_ds4(vid_pid)) {
				ksceBtStopInquiry();
				ds4_mac0 = hid_event.mac0;
				ds4_mac1 = hid_event.mac1;
			}
			break;
		}

		case 0x02: /* Inquiry stop event */
			if (!ds4_connected) {
				if (ds4_mac0 || ds4_mac1)
					ksceBtStartConnect(ds4_mac0, ds4_mac1);
			}
			break;

		case 0x04: /* Link key request? event */
			ksceBtReplyUserConfirmation(hid_event.mac0, hid_event.mac1, 1);
			break;

		case 0x05: /* Connection accepted event */
			ds4_input_reset();
			ds4_mac0 = hid_event.mac0;
			ds4_mac1 = hid_event.mac1;
			ds4_connected = 1;
			ds4_send_0x11_report(hid_event.mac0, hid_event.mac1);
			break;


		case 0x06: /* Device disconnect event*/
			ds4_connected = 0;
			reset_input_emulation();
			break;

		case 0x08: /* Connection requested event */
			/*
			 * Do nothing since we will get a 0x05 event afterwards.
			 */
			break;

		case 0x09: /* Connection request without being paired? event */
			/*
			 * The Vita needs to have a pairing with the DS4,
			 * otherwise it won't connect.
			 */
			break;

		case 0x0A: /* HID reply to 0-type request */

			LOG("DS4 0x0A event: 0x%02X\n", recv_buff[0]);

			switch (recv_buff[0]) {
			case 0x11:
				memcpy(&ds4_input, recv_buff, sizeof(ds4_input));

				set_input_emulation(&ds4_input);

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));
				break;

			default:
				LOG("Unknown DS4 event: 0x%02X\n", recv_buff[0]);
				break;
			}

			break;

		case 0x0B: /* HID reply to 1-type request */

			//LOG("DS4 0x0B event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;
		}
	}

	return 0;
}

static int ds4vita_bt_thread(SceSize args, void *argp)
{
	bt_cb_uid = ksceKernelCreateCallback("ds4vita_bt_callback", 0, bt_cb_func, NULL);
	LOG("Bluetooth callback UID: 0x%08X\n", bt_cb_uid);

	TEST_CALL(ksceBtRegisterCallback, bt_cb_uid, 0, 0xFFFFFFFF, 0xFFFFFFFF);

/*#ifndef RELEASE
	ksceBtStartInquiry();
	ksceKernelDelayThreadCB(4 * 1000 * 1000);
	ksceBtStopInquiry();
#endif*/

	while (bt_thread_run) {
		ksceKernelDelayThreadCB(200 * 1000);
	}

	if (ds4_connected) {
		ksceBtStartDisconnect(ds4_mac0, ds4_mac1);
		reset_input_emulation();
	}

	ksceBtUnregisterCallback(bt_cb_uid);

	ksceKernelDeleteCallback(bt_cb_uid);

	return 0;
}

void _start() __attribute__ ((weak, alias ("module_start")));

int module_start(SceSize argc, const void *args)
{
	int ret;
	tai_module_info_t SceBt_modinfo;

	log_reset();

	LOG("ds4vita by xerpi\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

	/* SceBt hooks */
	SceBt_sub_22999C8_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_22999C8_ref, SceBt_modinfo.modid, 0,
		0x22999C8 - 0x2280000, 1, SceBt_sub_22999C8_hook_func);

	/* SceTouch hooks */
	SceTouch_sceTouchPeek_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceTouch_sceTouchPeek_ref, "SceTouch", TAI_ANY_LIBRARY,
		0xFF082DF0, SceTouch_sceTouchPeek_hook_func);

	SceTouch_sceTouchPeekRegion_hook_uid = taiHookFunctionExportForKernel(KERNEL_PID,
		&SceTouch_sceTouchPeekRegion_ref, "SceTouch", TAI_ANY_LIBRARY,
		0x04440622, SceTouch_sceTouchPeekRegion_hook_func);

	SceKernelMemPoolCreateOpt opt;
	opt.size = 0x1C;
	opt.uselock = 0x100;
	opt.field_8 = 0x10000;
	opt.field_C = 0;
	opt.field_10 = 0;
	opt.field_14 = 0;
	opt.field_18 = 0;

	bt_mempool_uid = ksceKernelMemPoolCreate("ds4vita_mempool", 0x100, &opt);
	LOG("Bluetooth mempool UID: 0x%08X\n", bt_mempool_uid);

	bt_thread_uid = ksceKernelCreateThread("ds4vita_bt_thread", ds4vita_bt_thread,
		0x3C, 0x1000, 0, 0x10000, 0);
	LOG("Bluetooth thread UID: 0x%08X\n", bt_thread_uid);
	ksceKernelStartThread(bt_thread_uid, 0, NULL);

	LOG("module_start finished successfully!\n");

	return SCE_KERNEL_START_SUCCESS;

error_find_scebt:
	return SCE_KERNEL_START_FAILED;
}

int module_stop(SceSize argc, const void *args)
{
	SceUInt timeout = 0xFFFFFFFF;

	if (bt_thread_uid > 0) {
		bt_thread_run = 0;
		ksceKernelWaitThreadEnd(bt_thread_uid, NULL, &timeout);
		ksceKernelDeleteThread(bt_thread_uid);
	}

	if (bt_mempool_uid > 0) {
		ksceKernelMemPoolDestroy(bt_mempool_uid);
	}

	if (SceBt_sub_22999C8_hook_uid > 0) {
		taiHookReleaseForKernel(SceBt_sub_22999C8_hook_uid,
			SceBt_sub_22999C8_ref);
	}

	if (SceTouch_sceTouchPeek_hook_uid > 0) {
		taiHookReleaseForKernel(SceTouch_sceTouchPeek_hook_uid,
			SceTouch_sceTouchPeek_ref);
	}

	if (SceTouch_sceTouchPeekRegion_hook_uid > 0) {
		taiHookReleaseForKernel(SceTouch_sceTouchPeekRegion_hook_uid,
			SceTouch_sceTouchPeekRegion_ref);
	}

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
