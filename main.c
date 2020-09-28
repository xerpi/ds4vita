#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/bt.h>
#include <psp2kern/ctrl.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <taihen.h>
#include <math.h>
#include "log.h"
#include "ds34vita.h"

/*
 * Needed by newlib's libm.
 */
int __errno;

#define DS3_VID 0x054C
#define DS3_PID 0x0268

#define DS3_JOYSTICK_THRESHOLD 5
#define DS3_TRIGGER_THRESHOLD 1

#define DS4_VID   0x054C
#define DS4_PID   0x05C4
#define DS4_NEW_PID 0x09CC

#define DS4_TOUCHPAD_W 1920
#define DS4_TOUCHPAD_H 940
#define DS4_TOUCHPAD_W_DEAD 60
#define DS4_TOUCHPAD_H_DEAD 120
#define DS4_JOYSTICK_THRESHOLD 5
#define DS4_TRIGGER_THRESHOLD 0

#define VITA_FRONT_TOUCHSCREEN_W 1920
#define VITA_FRONT_TOUCHSCREEN_H 1080
#define VITA_PORTS_NUM 5

#define EVF_EXIT	(1 << 0)

#define abs(x) (((x) < 0) ? -(x) : (x))

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)

#define BIND_FUNC_OFFSET_HOOK(name, pid, modid, segidx, offset, thumb) \
	name##_hook_uid = taiHookFunctionOffsetForKernel((pid), \
		&name##_ref, (modid), (segidx), (offset), thumb, name##_hook_func)

#define BIND_FUNC_EXPORT_HOOK(name, pid, module, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionExportForKernel((pid), \
		&name##_ref, (module), (lib_nid), (func_nid), name##_hook_func)

#define UNBIND_FUNC_HOOK(name) \
	do { \
		if (name##_hook_uid > 0) \
			taiHookReleaseForKernel(name##_hook_uid, name##_ref); \
	} while(0)

struct ds3_input_report {
	unsigned char report_id;
	unsigned char unk0;

	unsigned char select : 1;
	unsigned char l3     : 1;
	unsigned char r3     : 1;
	unsigned char start  : 1;
	unsigned char up     : 1;
	unsigned char right  : 1;
	unsigned char down   : 1;
	unsigned char left   : 1;

	unsigned char l2       : 1;
	unsigned char r2       : 1;
	unsigned char l1       : 1;
	unsigned char r1       : 1;
	unsigned char triangle : 1;
	unsigned char circle   : 1;
	unsigned char cross    : 1;
	unsigned char square   : 1;

	unsigned char ps       : 1;
	unsigned char not_used : 7;

	unsigned char unk1;

	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned int unk2;

	unsigned char up_sens;
	unsigned char right_sens;
	unsigned char down_sens;
	unsigned char left_sens;

	unsigned char L2_sens;
	unsigned char R2_sens;
	unsigned char L1_sens;
	unsigned char R1_sens;

	unsigned char triangle_sens;
	unsigned char circle_sens;
	unsigned char cross_sens;
	unsigned char square_sens;

	unsigned short unk3;
	unsigned char unk4;

	unsigned char status;
	unsigned char power_rating;
	unsigned char comm_status;
	unsigned int unk5;
	unsigned int unk6;
	unsigned char unk7;

	unsigned short accel_x;
	unsigned short accel_y;
	unsigned short accel_z;

	union {
		unsigned short gyro_z;
		unsigned short roll;
	};
} __attribute__((packed, aligned(32)));

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

enum{
	SPACE_KERNEL,
	SPACE_USER,
	LOGIC_POSITIVE,
	LOGIC_NEGATIVE,
	TRIGGERS_EXT,
	TRIGGERS_NONEXT
};

typedef enum SceCtrlButtonsExt {
    SCE_CTRL_TOUCHPAD    = 0x04000000             //!< Dualshock 4 Touchpad button
} SceCtrlButtonsExt;

// Config
static int c_isExtAll = 0;
static int c_isPort1Allowed = 1;

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_evflag_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

typedef struct ControllerStats{
	int connected;
	int type;
	unsigned int mac0;
	unsigned int mac1;
	union{
		struct ds3_input_report report_ds3;
		struct ds4_input_report report_ds4;
	};
}ControllerStats;

static struct ControllerStats controllers[VITA_PORTS_NUM];

int32_t clamp(int32_t value, int32_t mini, int32_t maxi) {
	if (value < mini) return mini; 
	if (value > maxi) return maxi;
	return value;
}

static int get_free_port()
{
	for (int i = 1; i < VITA_PORTS_NUM; i++){
		if (i == 1 && !c_isPort1Allowed)
			continue;
		if (!controllers[i].connected){
			LOG("get_free_port() ret=%i\n", i);
			return i;
		}
	}
	return -1;
}

/*export*/ int ds34vita_getIsExtAll(){
	LOG("ds34vita_getIsExtAll()\n");
	return c_isExtAll;
}
/*export*/ void ds34vita_setIsExtAll(int/*bool*/ enabled){
	LOG("ds34vita_setIsExtAll(%i)\n", enabled);
	c_isExtAll = enabled;
}
/*export*/ int ds34vita_getIsPort1Allowed(){
	LOG("ds34vita_getIsPort1Allowed()\n");
	return c_isPort1Allowed;
}
/*export*/ void ds34vita_setIsPort1Allowed(int/*bool*/ enabled){
	LOG("ds34vita_setIsPort1Allowed(%i)\n", enabled);
	c_isPort1Allowed = enabled;
	if (enabled && !controllers[1].connected){
		for (int port = VITA_PORTS_NUM - 1; port > 1; port--){
			if (controllers[port].connected){
				controllers[1] = controllers[port];
				controllers[port].connected = 
					controllers[port].mac0 = 
					controllers[port].mac1 = 
					controllers[port].type = 0;
				break;
			}
		}
	} else if (!enabled && controllers[1].connected){
		int port = get_free_port();
		if(port > 0){
			controllers[port] = controllers[1];
		}
		controllers[1].connected = 
			controllers[1].mac0 = 
			controllers[1].mac1 = 
			controllers[1].type = 0;
	}
}

static inline void ds3_input_reset(ControllerStats* c)
{
	memset(&c->report_ds3, 0, sizeof(c->report_ds3));
}

static void ds4_input_reset(ControllerStats* c)
{
	memset(&c->report_ds4, 0, sizeof(c->report_ds4));
}

static int is_ds3(const unsigned short vid_pid[2])
{
	return vid_pid[0] == DS3_VID && vid_pid[1] == DS3_PID;
}

static int is_ds4(const unsigned short vid_pid[2])
{
	return (vid_pid[0] == DS4_VID) &&
		((vid_pid[1] == DS4_PID) || (vid_pid[1] == DS4_NEW_PID));
}

static inline void *mempool_alloc(unsigned int size)
{
	return ksceKernelAllocHeapMemory(bt_mempool_uid, size);
}

static inline void mempool_free(void *ptr)
{
	ksceKernelFreeHeapMemory(bt_mempool_uid, ptr);
}

static int ds3_send_feature_report(unsigned int mac0, unsigned int mac1, uint8_t flags, uint8_t report,
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
	req->type = 3; // 0x53 -> type = 3
	req->buffer = buf;
	req->length = len + 1;
	req->next = req;

	ksceBtHidTransfer(mac0, mac1, req);

	mempool_free(buf);
	mempool_free(req);

	return 0;
}

static int ds3_set_operational(unsigned int mac0, unsigned int mac1)
{
	unsigned char data[] = {
		0x42, 0x03, 0x00, 0x00
	};

	if (ds3_send_feature_report(mac0, mac1, 0, 0xF4, sizeof(data), data)) {
		LOG("Set operational error\n");
		return -1;
	}

	return 0;
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

	ksceBtHidTransfer(mac0, mac1, req);

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

DECL_FUNC_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo, SceCtrlPortInfo *info)
{
	int ret = TAI_CONTINUE(int, SceCtrl_ksceCtrlGetControllerPortInfo_ref, info);

	if (ret < 0)
		return ret;

	for (int i = 0; i < VITA_PORTS_NUM; i++){
		if (controllers[i].connected){
			info->port[i] = SCE_CTRL_TYPE_DS4;
		}
	}

	return ret;
}

DECL_FUNC_HOOK(SceCtrl_sceCtrlGetBatteryInfo, int port, SceUInt8 *batt)
{
	int ret = TAI_CONTINUE(int, SceCtrl_sceCtrlGetBatteryInfo_ref, port, batt);

	if (controllers[port].connected && controllers[port].type == SCE_CTRL_TYPE_DS4) {
		SceUInt8 k_batt;
		ksceKernelMemcpyUserToKernel(&k_batt, (uintptr_t)batt, sizeof(k_batt));

		if (controllers[port].report_ds4.usb_plugged) {
			k_batt = controllers[port].report_ds4.battery_level <= 10 ? 0xEE : 0xEF;
		} else {
			if (controllers[port].report_ds4.battery_level == 0)
				k_batt = 0;
			else
				k_batt = (controllers[port].report_ds4.battery_level / 2) + 1;

			if (k_batt > 5)
				k_batt = 5;
		}

		ksceKernelMemcpyKernelToUser((uintptr_t)batt, &k_batt, sizeof(k_batt));
		return 0;
	}

	return ret;
}

static void patch_ctrl_data_ds4(const struct ds4_input_report *ds4, SceCtrlData *pad_data, int port, int logic, int triggers)
{
	signed char ldx, ldy, rdx, rdy;
	unsigned int buttons = 0;
	int left_js_moved = 0;
	int right_js_moved = 0;

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

	if (triggers == TRIGGERS_EXT){
		if (ds4->l1)
			buttons |= SCE_CTRL_L1;
		if (ds4->r1)
			buttons |= SCE_CTRL_R1;

		if (ds4->l2)
			buttons |= SCE_CTRL_LTRIGGER;
		if (ds4->r2)
			buttons |= SCE_CTRL_RTRIGGER;

		if (ds4->l3)
			buttons |= SCE_CTRL_L3;
		if (ds4->r3)
			buttons |= SCE_CTRL_R3;

		if (ds4->l_trigger > DS4_TRIGGER_THRESHOLD)
			pad_data->lt = ds4->l_trigger;

		if (ds4->r_trigger > DS4_TRIGGER_THRESHOLD)
			pad_data->rt = ds4->r_trigger;
	} else {
		if (ds4->l1)
			buttons |= SCE_CTRL_LTRIGGER;
		if (ds4->r1)
			buttons |= SCE_CTRL_RTRIGGER;
		
		if (c_isExtAll){
			if (ds4->l2)
				buttons |= SCE_CTRL_L1;
			if (ds4->r2)
				buttons |= SCE_CTRL_R1;

			if (ds4->l3)
				buttons |= SCE_CTRL_L3;
			if (ds4->r3)
				buttons |= SCE_CTRL_R3;
		}
	}

	if (ds4->share)
		buttons |= SCE_CTRL_SELECT;
	if (ds4->options)
		buttons |= SCE_CTRL_START;
	
	if (c_isExtAll){
		if (ds4->tpad)
			buttons |= SCE_CTRL_TOUCHPAD;
	}

	ldx = ds4->left_x - 127;
	ldy = ds4->left_y - 127;
	rdx = ds4->right_x - 127;
	rdy = ds4->right_y - 127;

	if (sqrtf(ldx * ldx + ldy * ldy) > DS4_JOYSTICK_THRESHOLD)
		left_js_moved = 1;

	if (sqrtf(rdx * rdx + rdy * rdy) > DS4_JOYSTICK_THRESHOLD)
		right_js_moved = 1;

	if (port != 0)
		pad_data->lx = pad_data->ly = pad_data->rx = pad_data->ry = 127;

	if (left_js_moved) {
		pad_data->lx = clamp(pad_data->lx + ds4->left_x - 127, 0, 255);
		pad_data->ly = clamp(pad_data->ly + ds4->left_y - 127, 0, 255);
	}

	if (right_js_moved) {
		pad_data->rx = clamp(pad_data->rx + ds4->right_x - 127, 0, 255);
		pad_data->ry = clamp(pad_data->ry + ds4->right_y - 127, 0, 255);
	}

	if (ds4->ps)
		ksceCtrlSetButtonEmulation(0, 0, 0, SCE_CTRL_INTERCEPTED, 16);

	if (buttons != 0 || left_js_moved || right_js_moved ||
	    ds4->l_trigger > DS4_TRIGGER_THRESHOLD ||
	    ds4->r_trigger > DS4_TRIGGER_THRESHOLD)
		ksceKernelPowerTick(0);

	if (logic == LOGIC_NEGATIVE)
		pad_data->buttons = 0xFFFFFFFF - pad_data->buttons;
	if (port != 0)
		pad_data->buttons = 0;

	pad_data->buttons |= buttons;
	
	if (logic == LOGIC_NEGATIVE)
		pad_data->buttons = 0xFFFFFFFF - pad_data->buttons;
}

static void patch_ctrl_data_ds3(const struct ds3_input_report *ds3, SceCtrlData *pad_data, int port, int logic, int triggers)
{
	signed char ldx, ldy, rdx, rdy;
	unsigned int buttons = 0;
	int left_js_moved = 0;
	int right_js_moved = 0;

	if (ds3->cross)
		buttons |= SCE_CTRL_CROSS;
	if (ds3->circle)
		buttons |= SCE_CTRL_CIRCLE;
	if (ds3->triangle)
		buttons |= SCE_CTRL_TRIANGLE;
	if (ds3->square)
		buttons |= SCE_CTRL_SQUARE;

	if (ds3->up)
		buttons |= SCE_CTRL_UP;
	if (ds3->right)
		buttons |= SCE_CTRL_RIGHT;
	if (ds3->down)
		buttons |= SCE_CTRL_DOWN;
	if (ds3->left)
		buttons |= SCE_CTRL_LEFT;

	
	if (triggers == TRIGGERS_EXT){
		if (ds3->l1)
			buttons |= SCE_CTRL_L1;
		if (ds3->r1)
			buttons |= SCE_CTRL_R1;

		if (ds3->l2)
			buttons |= SCE_CTRL_LTRIGGER;
		if (ds3->r2)
			buttons |= SCE_CTRL_RTRIGGER;

		if (ds3->l3)
			buttons |= SCE_CTRL_L3;
		if (ds3->r3)
			buttons |= SCE_CTRL_R3;

		if (ds3->L2_sens > DS3_TRIGGER_THRESHOLD)
			pad_data->lt = ds3->L2_sens;

		if (ds3->R2_sens > DS3_TRIGGER_THRESHOLD)
			pad_data->rt = ds3->R2_sens;

	} else {
		if (ds3->l1)
			buttons |= SCE_CTRL_LTRIGGER;
		if (ds3->r1)
			buttons |= SCE_CTRL_RTRIGGER;
			
		if (c_isExtAll){
			if (ds3->l2)
				buttons |= SCE_CTRL_L1;
			if (ds3->r2)
				buttons |= SCE_CTRL_R1;

			if (ds3->l3)
				buttons |= SCE_CTRL_L3;
			if (ds3->r3)
				buttons |= SCE_CTRL_R3;
		}
	}

	if (ds3->select)
		buttons |= SCE_CTRL_SELECT;
	if (ds3->start)
		buttons |= SCE_CTRL_START;
	if (ds3->ps)
		buttons |= SCE_CTRL_INTERCEPTED;

	ldx = ds3->left_x - 128;
	ldy = ds3->left_y - 128;
	rdx = ds3->right_x - 128;
	rdy = ds3->right_y - 128;

	if (sqrtf(ldx * ldx + ldy * ldy) > DS3_JOYSTICK_THRESHOLD)
		left_js_moved = 1;

 	if (sqrtf(rdx * rdx + rdy * rdy) > DS3_JOYSTICK_THRESHOLD)
		right_js_moved = 1;

	if (port != 0)
		pad_data->lx = pad_data->ly = pad_data->rx = pad_data->ry = 127;

	if (left_js_moved) {
		pad_data->lx = clamp(pad_data->lx + ds3->left_x - 127, 0, 255);
		pad_data->ly = clamp(pad_data->ly + ds3->left_y - 127, 0, 255);
	}

	if (right_js_moved) {
		pad_data->rx = clamp(pad_data->rx + ds3->right_x - 127, 0, 255);
		pad_data->ry = clamp(pad_data->ry + ds3->right_y - 127, 0, 255);
	}

	if (ds3->ps)
		ksceCtrlSetButtonEmulation(0, 0, 0, SCE_CTRL_INTERCEPTED, 16);

	if (buttons != 0 || left_js_moved || right_js_moved ||
	    ds3->L2_sens > DS3_TRIGGER_THRESHOLD ||
	    ds3->R2_sens > DS3_TRIGGER_THRESHOLD)
		ksceKernelPowerTick(0);

	if (logic == LOGIC_NEGATIVE)
		pad_data->buttons = 0xFFFFFFFF - pad_data->buttons;
	if (port != 0)
		pad_data->buttons = 0;

	pad_data->buttons |= buttons;
	
	if (logic == LOGIC_NEGATIVE)
		pad_data->buttons = 0xFFFFFFFF - pad_data->buttons;
}

static void patch_ctrl_data_all(const struct ControllerStats* controller,
				     int port, SceCtrlData *pad_data, int count,
					 int space, int logic, int triggers)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		if (space == SPACE_USER){
			SceCtrlData k_data;
			ksceKernelMemcpyUserToKernel(&k_data, (uintptr_t)pad_data, sizeof(k_data));
			if (controller->type == SCE_CTRL_TYPE_DS4)
				patch_ctrl_data_ds4(&controller->report_ds4, &k_data, port, logic, triggers);
			else 
				patch_ctrl_data_ds3(&controller->report_ds3, &k_data, port, logic, triggers);
			ksceKernelMemcpyKernelToUser((uintptr_t)pad_data, &k_data, sizeof(k_data));
		} else {
			if (controller->type == SCE_CTRL_TYPE_DS4)
				patch_ctrl_data_ds4(&controller->report_ds4, pad_data, port, logic, triggers);
			else 
				patch_ctrl_data_ds3(&controller->report_ds3, pad_data, port, logic, triggers);
		}
		pad_data++;
	}
}

#define DECL_FUNC_HOOK_PATCH_CTRL(name, space, logic, triggers) \
	DECL_FUNC_HOOK(SceCtrl_##name, int port, SceCtrlData *pad_data, int count) \
	{ \
		int ret; \
		if (port == 0 && controllers[1].connected){ \
			ret = TAI_CONTINUE(int, SceCtrl_ ##name##_ref, 0, pad_data, count); \
			patch_ctrl_data_all(&controllers[1], port, pad_data, count, (space), (logic), (triggers)); \
			return ret; \
		} else if (controllers[port].connected){ \
			ret = TAI_CONTINUE(int, SceCtrl_ ##name##_ref, 0, pad_data, count); \
			patch_ctrl_data_all(&controllers[port], port, pad_data, count, (space), (logic), (triggers)); \
			return ret; \
		} \
		return TAI_CONTINUE(int, SceCtrl_ ##name##_ref, port, pad_data, count); \
	} \

DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlPeekBufferPositiveExt,  SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_NONEXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlReadBufferPositiveExt,  SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_NONEXT)

DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlPeekBufferPositive2,    SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_EXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlReadBufferPositive2,    SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_EXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlPeekBufferNegative2,    SPACE_USER,   LOGIC_NEGATIVE, TRIGGERS_EXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlReadBufferNegative2,    SPACE_USER,   LOGIC_NEGATIVE, TRIGGERS_EXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlPeekBufferPositiveExt2, SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_EXT)
DECL_FUNC_HOOK_PATCH_CTRL(sceCtrlReadBufferPositiveExt2, SPACE_USER,   LOGIC_POSITIVE, TRIGGERS_EXT)

DECL_FUNC_HOOK_PATCH_CTRL(ksceCtrlPeekBufferPositive,    SPACE_KERNEL, LOGIC_POSITIVE, TRIGGERS_NONEXT)
DECL_FUNC_HOOK_PATCH_CTRL(ksceCtrlReadBufferPositive,    SPACE_KERNEL, LOGIC_POSITIVE, TRIGGERS_NONEXT)
DECL_FUNC_HOOK_PATCH_CTRL(ksceCtrlPeekBufferNegative,    SPACE_KERNEL, LOGIC_NEGATIVE, TRIGGERS_NONEXT)
DECL_FUNC_HOOK_PATCH_CTRL(ksceCtrlReadBufferNegative,    SPACE_KERNEL, LOGIC_NEGATIVE, TRIGGERS_NONEXT)

static int rescaleTouchCoordinate(int ds4coord, int ds4size, int ds4dead, int vitasize){
	return clamp(((ds4coord - ds4dead) * vitasize) / (ds4size - ds4dead * 2), 0, vitasize - 1);
}

static void patch_touch_data(SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs,
			    struct ds4_input_report *ds4)
{
	unsigned int i;

	if (port != SCE_TOUCH_PORT_FRONT)
		return;

	for (i = 0; i < nBufs; i++) {
		unsigned int num_reports = 0;

		// If finger1 present, add finger1 as report 0
		if (!ds4->finger1_activelow) {
			pData[i].report[0].id = ds4->finger1_id;
			pData[i].report[0].x = rescaleTouchCoordinate(ds4->finger1_x, DS4_TOUCHPAD_W, DS4_TOUCHPAD_W_DEAD, VITA_FRONT_TOUCHSCREEN_W);
			pData[i].report[0].y = rescaleTouchCoordinate(ds4->finger1_y, DS4_TOUCHPAD_H, DS4_TOUCHPAD_H_DEAD, VITA_FRONT_TOUCHSCREEN_H);
			num_reports++;
		}

		// If only finger2 is present, add finger2 as report 0
		if (!ds4->finger2_activelow && ds4->finger1_activelow) {
			pData[i].report[0].id = ds4->finger2_id;
			pData[i].report[0].x = rescaleTouchCoordinate(ds4->finger2_x, DS4_TOUCHPAD_W, DS4_TOUCHPAD_W_DEAD, VITA_FRONT_TOUCHSCREEN_W);
			pData[i].report[0].y = rescaleTouchCoordinate(ds4->finger2_y, DS4_TOUCHPAD_H, DS4_TOUCHPAD_H_DEAD, VITA_FRONT_TOUCHSCREEN_H);
			num_reports++;
		}

		// If both finger1 and finger2 present, add finger2 as report 1
		if (!ds4->finger2_activelow && !ds4->finger1_activelow) {
			pData[i].report[1].id = ds4->finger2_id;
			pData[i].report[1].x = rescaleTouchCoordinate(ds4->finger2_x, DS4_TOUCHPAD_W, DS4_TOUCHPAD_W_DEAD, VITA_FRONT_TOUCHSCREEN_W);
			pData[i].report[1].y = rescaleTouchCoordinate(ds4->finger2_y, DS4_TOUCHPAD_H, DS4_TOUCHPAD_H_DEAD, VITA_FRONT_TOUCHSCREEN_H);
			num_reports++;
		}

		if (num_reports > 0) {
			ksceKernelPowerTick(0);
			pData->reportNum = num_reports;
		}

		pData++;
	}
}

DECL_FUNC_HOOK(SceTouch_ksceTouchPeek, SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs)
{
	int ret = TAI_CONTINUE(int, SceTouch_ksceTouchPeek_ref, port, pData, nBufs);
	
	if (ret >= 0 && controllers[1].connected && controllers[1].type == SCE_CTRL_TYPE_DS4)
		patch_touch_data(port, pData, nBufs, &controllers[1].report_ds4);

	return ret;
}

DECL_FUNC_HOOK(SceTouch_ksceTouchPeekRegion, SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs, int region)
{
	int ret = TAI_CONTINUE(int, SceTouch_ksceTouchPeekRegion_ref, port, pData, nBufs, region);

	if (ret >= 0 && controllers[1].connected && controllers[1].type == SCE_CTRL_TYPE_DS4)
		patch_touch_data(port, pData, nBufs, &controllers[1].report_ds4);

	return ret;
}

DECL_FUNC_HOOK(SceTouch_ksceTouchRead, SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs)
{
	int ret = TAI_CONTINUE(int, SceTouch_ksceTouchRead_ref, port, pData, nBufs);

	if (ret >= 0 && controllers[1].connected && controllers[1].type == SCE_CTRL_TYPE_DS4)
		patch_touch_data(port, pData, nBufs, &controllers[1].report_ds4);

	return ret;
}

DECL_FUNC_HOOK(SceTouch_ksceTouchReadRegion, SceUInt32 port, SceTouchData *pData, SceUInt32 nBufs, int region)
{
	int ret = TAI_CONTINUE(int, SceTouch_ksceTouchReadRegion_ref, port, pData, nBufs, region);

	if (ret >= 0 && controllers[1].connected && controllers[1].type == SCE_CTRL_TYPE_DS4)
		patch_touch_data(port, pData, nBufs, &controllers[1].report_ds4);

	return ret;
}

static void patch_motion_state(SceMotionState *motionState, struct ds4_input_report *ds4)
{
	SceMotionState k_data;
	SceMotionState *u_data = motionState;

	ksceKernelMemcpyUserToKernel(&k_data, (uintptr_t)u_data, sizeof(k_data));
	k_data.acceleration.x = ds4->accel_x;
	k_data.acceleration.y = ds4->accel_y;
	k_data.acceleration.y = ds4->accel_z;
	ksceKernelMemcpyKernelToUser((uintptr_t)u_data, &k_data, sizeof(k_data));
}

DECL_FUNC_HOOK(SceMotion_sceMotionGetState, SceMotionState *motionState)
{
	int ret = TAI_CONTINUE(int, SceMotion_sceMotionGetState_ref, motionState);

	if (ret >= 0 && controllers[1].connected && controllers[1].type == SCE_CTRL_TYPE_DS4)
		patch_motion_state(motionState, &controllers[1].report_ds4);

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

DECL_FUNC_HOOK(SceBt_sub_22999C8, void *dev_base_ptr, int r1)
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

static SceUID SceBt_sub_22947E4_hook_uid = -1;
static tai_hook_ref_t SceBt_sub_22947E4_ref;
static void *SceBt_sub_22947E4_hook_func(unsigned int r0, unsigned int r1, unsigned long long r2)
{
	void *ret = TAI_CONTINUE(void *, SceBt_sub_22947E4_ref, r0, r1, r2);

	if (ret) {
		/*
		 * We have to enable this bit in order to make the Vita
		 * accept the controller.
		 */
		*(unsigned int *)(ret + 0x24) |= 0x1000;
	}

	return ret;
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

		LOG("\n");
		LOG("->Event:");
		LOGF("%02X: ", hid_event.id);
		for (int i = 0; i < 0x10; i++)
			LOGF(" %02X", hid_event.data[i]);
		LOGF("\n");

		ControllerStats *controller = NULL;
		// Find connected controllers with same mac
		for (int i = 0; i < VITA_PORTS_NUM; i++){
			if (controllers[i].connected && controllers[i].mac0 == hid_event.mac0 && controllers[i].mac1 == hid_event.mac1){
				controller = &controllers[i];
				LOG("[%08X, %08X]>[%08X, %08X] Using controller from port %i\n", 
					controllers[i].mac0, controllers[i].mac1, hid_event.mac0, hid_event.mac1, i);
			}
		}
		// Find free port
		int port = -1;
		if (controller == NULL){
			port = get_free_port();
			if (port < 0){
				LOG("No free ports \n");
				continue;
			}
			LOG("[%08X, %08X]>[%08X, %08X] Using free port %i\n", 
				controllers[port].mac0, controllers[port].mac1, hid_event.mac0, hid_event.mac1, port);
			controller = &controllers[port];
		}

		switch (hid_event.id) {
		case 0x01: { /* Inquiry result event */
			unsigned short vid_pid[2];
			ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);

			if (is_ds4(vid_pid)) {
				ksceBtStopInquiry();
				controller->mac0 = hid_event.mac0;
				controller->mac1 = hid_event.mac1;
			}
			break;
		}

		case 0x02: /* Inquiry stop event */
			if (!controller->connected && controller->type == SCE_CTRL_TYPE_DS4) {
				if (controller->mac0 || controller->mac1)
					ksceBtStartConnect(controller->mac0, controller->mac1);
			}
			break;

		case 0x04: /* Link key request? event */
			ksceBtReplyUserConfirmation(hid_event.mac0, hid_event.mac1, 1);
			break;

		case 0x05: { /* Connection accepted event */
			unsigned short vid_pid[2];
			char name[0x79];
			unsigned int result1;
			unsigned int result2;

			result1 = ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);
			result2 = ksceBtGetDeviceName(hid_event.mac0, hid_event.mac1, name);

			if (is_ds4(vid_pid)) {
				ds4_input_reset(controller);
				controller->mac0 = hid_event.mac0;
				controller->mac1 = hid_event.mac1;
				controller->connected = 1;
				controller->type = SCE_CTRL_TYPE_DS4;
				ds4_send_0x11_report(hid_event.mac0, hid_event.mac1);
				LOG("     CONNECTED DS3[%08X %08X] TO PORT %i\n", hid_event.mac0, hid_event.mac1, port);
			} else if (is_ds3(vid_pid) || (result1 == 0x802F5001 &&
			    	result2 == 0x802F0C01)){
				controller->mac0 = hid_event.mac0;
				controller->mac1 = hid_event.mac1;
				controller->connected = 1;
				controller->type = SCE_CTRL_TYPE_DS3;
				ds3_set_operational(hid_event.mac0, hid_event.mac1);
				LOG("     CONNECTED DS4[%08X %08X] TO PORT %i\n", hid_event.mac0, hid_event.mac1, port);
			}
			break;
		}

		case 0x06: /* Device disconnect event*/
			controller->connected = 0;
			controller->mac0 = 0;
			controller->mac1 = 0;
			if (controller->type == SCE_CTRL_TYPE_DS4)
				ds4_input_reset(controller);
			else 
				ds3_input_reset(controller);
			controller->type = 0;
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

			LOG("0x0A event: 0x%02X\n", recv_buff[0]);

			switch (recv_buff[0]) {
			case 0x01:
				/*
				 * Save DS3 state to a global variable.
				 */
				if (!controller->connected || !(controller->type == SCE_CTRL_TYPE_DS3))
					break;

				memcpy(&controller->report_ds3, recv_buff, sizeof(controller->report_ds3));

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));
				break;
			case 0x11:
				/*
				 * Save DS4 state to a global variable.
				 */
				if (!controller->connected || !(controller->type == SCE_CTRL_TYPE_DS4))
					break;

				memcpy(&controller->report_ds4, recv_buff, sizeof(controller->report_ds4));

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));
				break;

			default:
				LOG("Unknown event: 0x%02X\n", recv_buff[0]);
				break;
			}

			break;

		case 0x0B: /* HID reply to 1-type request */

			//LOG("DS4 0x0B event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;
		
		case 0x0C: /* HID reply to 3-type request? */

			//LOG("DS3 0x0C event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;
		}

	}

	return 0;
}

static int ds34vita_bt_thread(SceSize args, void *argp)
{
	bt_cb_uid = ksceKernelCreateCallback("ds34vita_bt_callback", 0, bt_cb_func, NULL);

	ksceBtRegisterCallback(bt_cb_uid, 0, 0xFFFFFFFF, 0xFFFFFFFF);

	while (bt_thread_run) {
		int ret;
		unsigned int evf_out;

		ret = ksceKernelWaitEventFlagCB(bt_thread_evflag_uid, EVF_EXIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR_PAT, &evf_out, NULL);
		if (ret < 0)
			continue;

		if (evf_out & EVF_EXIT)
			break;
	}

	for (int i = 0; i < VITA_PORTS_NUM; i++){
		if (controllers[i].connected)
			ksceBtStartDisconnect(controllers[i].mac0, controllers[i].mac1);
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
	memset(controllers, 0, sizeof(controllers));

	LOG("ds34vita by xerpi\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

	/* SceBt hooks */
	BIND_FUNC_OFFSET_HOOK(SceBt_sub_22999C8, KERNEL_PID,
		SceBt_modinfo.modid, 0, 0x22999C8 - 0x2280000, 1);

	
	SceBt_sub_22947E4_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_22947E4_ref, SceBt_modinfo.modid, 0,
		0x22947E4 - 0x2280000, 1, SceBt_sub_22947E4_hook_func);

	// BIND_FUNC_OFFSET_HOOK(SceBt_sub_22947E4, KERNEL_PID,
	// 	SceBt_modinfo.modid, 0, 0x22947E4 - 0x2280000, 1);

	/* Patch PAD Type */
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xF11D0D30);

	/* Patch Battery level */
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlGetBatteryInfo, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x8F9B1CE5);

	/* SceCtrl hooks:
	 * sceCtrlPeekBufferPositive -> ksceCtrlPeekBufferPositive
	 * sceCtrlReadBufferPositive -> ksceCtrlReadBufferPositive
	 * sceCtrlPeekBufferNegative -> ksceCtrlPeekBufferNegative
	 * sceCtrlReadBufferNegative -> ksceCtrlReadBufferNegative
	 * sceCtrlPeekBufferPositiveExt -> none
	 * sceCtrlReadBufferPositiveExt -> none
	 * 
	 * sceCtrlPeekBufferPositive2 -> none
	 * sceCtrlReadBufferPositive2 -> none
	 * sceCtrlPeekBufferNegative2 -> none
	 * sceCtrlReadBufferNegative2 -> none
	 * sceCtrlPeekBufferPositiveExt2 -> none
	 * sceCtrlReadBufferPositiveExt2 -> none
	 * 
	 * ksceCtrlPeekBufferPositive -> none
	 * ksceCtrlReadBufferPositive -> none
	 * ksceCtrlPeekBufferNegative -> none
	 * ksceCtrlReadBufferNegative -> none
	 */

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0xA59454D3);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt, KERNEL_PID,	"SceCtrl", TAI_ANY_LIBRARY, 0xE2D99296);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositive2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x15F81E8C);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositive2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0xC4226A3E);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferNegative2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x81A89660);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferNegative2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x27A0C5FB);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x860BF292);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt2, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0xA7178860);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlPeekBufferPositive, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0xEA1D3A34);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlReadBufferPositive, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x9B96A1AA);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlPeekBufferNegative, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x19895843);
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlReadBufferNegative, KERNEL_PID, "SceCtrl", TAI_ANY_LIBRARY, 0x8D4E0DD1);

	/* SceTouch hooks */
	BIND_FUNC_EXPORT_HOOK(SceTouch_ksceTouchPeek, KERNEL_PID, "SceTouch", TAI_ANY_LIBRARY, 0xBAD1960B);
	BIND_FUNC_EXPORT_HOOK(SceTouch_ksceTouchPeekRegion, KERNEL_PID,	"SceTouch", TAI_ANY_LIBRARY, 0x9B3F7207);
	BIND_FUNC_EXPORT_HOOK(SceTouch_ksceTouchRead, KERNEL_PID, "SceTouch", TAI_ANY_LIBRARY, 0x70C8AACE);
	BIND_FUNC_EXPORT_HOOK(SceTouch_ksceTouchReadRegion, KERNEL_PID,	"SceTouch", TAI_ANY_LIBRARY, 0x9A91F624);

	/* SceMotion hooks */
	BIND_FUNC_EXPORT_HOOK(SceMotion_sceMotionGetState, KERNEL_PID, "SceMotion", TAI_ANY_LIBRARY, 0xBDB32767);

	SceKernelHeapCreateOpt opt;
	opt.size = 0x1C;
	opt.uselock = 0x100;
	opt.field_8 = 0x10000;
	opt.field_C = 0;
	opt.field_10 = 0;
	opt.field_14 = 0;
	opt.field_18 = 0;

	bt_mempool_uid = ksceKernelCreateHeap("ds34vita_mempool", 0x100, &opt);
	LOG("Bluetooth mempool UID: 0x%08X\n", bt_mempool_uid);

	bt_thread_evflag_uid = ksceKernelCreateEventFlag("ds34vita_bt_thread_evflag",
							 0, 0, NULL);
	LOG("Bluetooth thread event flag UID: 0x%08X\n", bt_thread_evflag_uid);

	bt_thread_uid = ksceKernelCreateThread("ds34vita_bt_thread", ds34vita_bt_thread,
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

	bt_thread_run = 0;

	if (bt_thread_evflag_uid)
		ksceKernelSetEventFlag(bt_thread_evflag_uid, EVF_EXIT);

	if (bt_thread_uid > 0) {
		ksceKernelWaitThreadEnd(bt_thread_uid, NULL, &timeout);
		ksceKernelDeleteThread(bt_thread_uid);
	}

	if (bt_thread_evflag_uid)
		ksceKernelDeleteEventFlag(bt_thread_evflag_uid);

	if (bt_mempool_uid > 0) {
		ksceKernelDeleteHeap(bt_mempool_uid);
	}

	UNBIND_FUNC_HOOK(SceBt_sub_22999C8);

	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlGetBatteryInfo);

	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt);

	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositive2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositive2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferNegative2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferNegative2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt2);

	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlReadBufferNegative);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlPeekBufferPositive);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlReadBufferPositive);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlPeekBufferNegative);

	UNBIND_FUNC_HOOK(SceTouch_ksceTouchPeek);
	UNBIND_FUNC_HOOK(SceTouch_ksceTouchPeekRegion);
	UNBIND_FUNC_HOOK(SceTouch_ksceTouchRead);
	UNBIND_FUNC_HOOK(SceTouch_ksceTouchReadRegion);

	UNBIND_FUNC_HOOK(SceMotion_sceMotionGetState);

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
