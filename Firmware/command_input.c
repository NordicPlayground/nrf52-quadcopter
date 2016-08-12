#include "command_input.h"
#include "include_common.h"
#include "command_input.h"

enum {CONTROL,COMMAND,TUNE};
#define COMMAND_LEN 5
#define CONTROL_LEN 5
#define TUNE_LEN 5

#define PRESCALER 0
#define RUN_DUMMY 0
#define THROTTLE_SCALER 257.0f //scale to uint16_t from uint8_t
APP_TIMER_DEF(dummy_timer_id);

static void remote_control_dummy(void * arg);

static attitude stab_setpoints;

float throttle_setpoint = 0;
//static bool throttle_data_is_new = false; Set, but never used. 1 Warning.
//static bool stab_data_is_new = false; Set, but never used. 1 Warning.

static void remote_control_dummy_init(void);

uint32_t cmd_init(){
    NRF_LOG("cmd_init()\n");
    memset(&stab_setpoints,0,sizeof(stab_setpoints));
    memset(&throttle_setpoint,0,sizeof(throttle_setpoint));

    if (RUN_DUMMY){
        remote_control_dummy_init();
    }
    return NRF_SUCCESS;
}

attitude cmd_get_setpoints(void){
    //stab_data_is_new = false;
    return stab_setpoints;
}

void set_attitude_setpoints(uint8_t * controldata){
	throttle_setpoint = (float)controldata[0]*THROTTLE_SCALER;
	stab_setpoints.roll = -((float)controldata[1] - (float)controldata[2]);
	stab_setpoints.pitch = ((float)controldata[3] - (float)controldata[4]);
	stab_setpoints.yaw = ((float)controldata[5] - (float)controldata[6]);
}

float cmd_get_throttle(void){
    //throttle_data_is_new = false;
    return throttle_setpoint;
}

/*static void print_packet(uint8_t *packet, uint8_t length){ ********Declared, but never referenced.
    for (int i=0;i<length;i++){
        NRF_LOG("packet[%d]: %d",i,packet[i]);
    }
    NRF_LOG("\n");
}*/

void cmd_on_ble_write(uint8_t *data, uint16_t length){
    /*//Check type of packet (control or other command)
    //check validity
    //NRF_LOG_PRINTF("received length: %d \n",length);
    //command packet:
        //dispatch to correct function

    //control packet:
        //convert to stabilizer readable range

    NRF_LOG_PRINTF("received packet: %s\n",data);
    print_packet(data,(uint8_t)length);
    if ((data[0] == CONTROL) && (length == CONTROL_LEN)){ // data is 8 bit signed
        float temp = (float)data[1];
        ///NRF_LOG_PRINTF("data[1]: %d  ", data[1]);
        temp -=JOYSTICK_MIDDLEPOINT;
        if (temp <0){
            temp = 0;
        }
        throttle_setpoint = temp*THROTTLE_SCALER; //scale to uint16_t value from uint8_t value
        stab_setpoints.roll = (float)(data[2]-JOYSTICK_MIDDLEPOINT);
        stab_setpoints.pitch = (float)(data[3]-JOYSTICK_MIDDLEPOINT);
        stab_setpoints.yaw = (float)(data[4]-JOYSTICK_MIDDLEPOINT);
        stab_data_is_new = true;
        throttle_data_is_new = true;
    }

    //received packet is a command
    else if ((data[0] == COMMAND) && (length == COMMAND_LEN)){

    }
    //received packet is tuning
    else if ((data[0] == TUNE) && (length == TUNE_LEN)){

    }*/
}

static void remote_control_dummy(void * arg){
    NRF_LOG("remote_control_dummy task\n");
    uint8_t packet_length = 8;
    uint8_t control[packet_length];
    uint8_t i = 0;

    for (int j = 0;j<packet_length;j++){
        //i++;
        control[j]=i;
    }
    cmd_on_ble_write(control, packet_length);

}

static void remote_control_dummy_init(void){
    uint32_t err_code;
    NRF_LOG("remote_control_dummy init\n");
    err_code = app_timer_create(&dummy_timer_id, APP_TIMER_MODE_REPEATED, remote_control_dummy);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(dummy_timer_id, APP_TIMER_TICKS(1000, PRESCALER),NULL);
    APP_ERROR_CHECK(err_code);
    return;
}

euler_angle map(euler_angle input){
    if ((CMD_INPUT_MIN == CMD_OUTPUT_MIN) && (CMD_INPUT_MAX == CMD_OUTPUT_MAX)){
        return input;
    }
    return ((input-CMD_INPUT_MIN) * (CMD_OUTPUT_MAX-CMD_OUTPUT_MIN)) / (CMD_INPUT_MAX-CMD_INPUT_MIN) +CMD_OUTPUT_MIN;
}
