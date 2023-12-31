#include <Servo.h>
#include <Arduino_APDS9960.h>

#include "motion.h"
#include "variant.h"

#define US_TRIG_PIN         7
#define US_ECHO_PIN         6

// XXX
#define IR_LEFT_INT_PIN     0
#define IR_RIGHT_INT_PIN    0

// 50ms no mínimo para cada "passo"/"etapa"
#define MIN_TICKS_PER_STEP 50
#define DEF_TICKS_PER_STEP (MIN_TICKS_PER_STEP + 50)

// O mínimo é 10ms+d (tempo para acionar o sensor ultrassônico), a macro a seguir define o valor de d (em ms)
#define SENSOR_REFRESH_INTERVAL 20

// "Feature flags"
// #define DEBUG
#define BLUETOOTH
#define IR_SENSOR
#define US_SENSOR

#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_IS_NOT_COMM_SERIAL true

#define SIZEOF_ARRAY(x) (sizeof (x) / sizeof ((x)[0]))

static uint8_t ticks_per_step = DEF_TICKS_PER_STEP;

// Da mais interna à mais externa,
// U - upper, M - middle, R - rear
enum servo_pos {
    HEAD = 0,
    HEAD_IN = 1,
    GRIP = 2,
    TAIL = 3,
    LEG_UL = 4,
    LEG_ML = 7,
    LEG_RL = 10,
    LEG_UR = 13,
    LEG_MR = 16,
    LEG_RR = 19
};

enum servo_off {
    REST = servo_pos::HEAD,
    TORAX = servo_pos::LEG_UL
};

enum commands {
    CMD_MOVE_FWD,
    CMD_MOVE_REV,
    CMD_MOVE_ROT_LEFT,
    CMD_MOVE_ROT_RIGHT,
    CMD_BITE,
    CMD_ATTACK,
    CMD_GRAB,
    CMD_DROP,
    CMD_WAGTAIL,
    CMD_SHAKEHEAD,
    /* Altera o estado atual do piloto automático */
    CMD_AUTO,
    CMD_SET_SPEED
};

bool is_auto = false;
Servo servos[21];

static step_body_t step_body = { nullptr, 0, 0, 0 };
static step_rest_t step_rest = { nullptr, 0, 0 };
static step_robot_t step_robot = { nullptr, 0, 0 };

bool timer_has_run = false;
uint32_t time_last_step = -1;

#ifdef IR_SENSOR
APDS9960 sensor_left(Wire, IR_LEFT_INT_PIN);
APDS9960 sensor_right(Wire1, IR_RIGHT_INT_PIN);
#endif

struct {
#ifdef IR_SENSOR
    uint8_t left, right;
#endif
#ifdef US_SENSOR
    uint8_t front;
#endif
} sensor_data;

uint8_t time_last_meas = -1;
bool sent_meas_trig = false;

void setup() {
#ifdef BLUETOOTH
    Serial2.begin(9600);
#endif

    while (Serial2.available())
        Serial2.read();

#if defined(DEBUG) && defined(DEBUG_SERIAL_IS_NOT_COMM_SERIAL)
    DEBUG_SERIAL.begin(9600);
#endif

#ifdef IR_SENSOR
    bool res;
    
    res = sensor_left.begin();

#ifdef DEBUG
    if (!res)
        DEBUG_SERIAL.println("DBG: Failed to init left IR sensor");
#endif

    res = sensor_right.begin();

#ifdef DEBUG
    if (!res)
        DEBUG_SERIAL.println("DBG: Failed to init right IR sensor");
#endif
#endif /* IR_SENSOR */

#ifdef US_SENSOR
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
#endif

    for (int i = 0; i <= SIZEOF_ARRAY(servos); i++) {
        servos[i].attach(i + 22);
        servos[i].write(90);
    }

    delay(3000);
}

void execute_command(commands cmd, int speed) {
    switch (cmd) {
        case CMD_MOVE_FWD:
            if (step_body.step_seq != &forward[0] || step_body.seq_dir != +1) {
                step_body.step_seq = &forward[0];
                step_body.steps = SIZEOF_ARRAY(forward);
                step_body.seq_dir = +1;
            }
            break;
        case CMD_MOVE_REV:
            if (step_body.step_seq != &forward[0] || step_body.seq_dir != -1) {
                step_body.step_seq = &forward[0];
                step_body.steps = SIZEOF_ARRAY(forward);
                step_body.seq_dir = -1;
            }
            break;
        case CMD_MOVE_ROT_LEFT:
            if (step_body.step_seq != &rotate_cw[0] || step_body.seq_dir != -1) {
                step_body.step_seq = &rotate_cw[0];
                step_body.steps = SIZEOF_ARRAY(rotate_cw);
                step_body.seq_dir = -1;
            }
            break;
        case CMD_MOVE_ROT_RIGHT:
            if (step_body.step_seq != &rotate_cw[0] || step_body.seq_dir != +1) {
                step_body.step_seq = &rotate_cw[0];
                step_body.steps = SIZEOF_ARRAY(rotate_cw);
                step_body.seq_dir = +1;
            }
            break;
        case CMD_BITE:
            step_rest.step_seq = &bite[0];
            step_rest.steps = SIZEOF_ARRAY(bite);
            break;
        case CMD_ATTACK:
            break;
        case CMD_GRAB:
            step_rest.step_seq = &grab[0];
            step_rest.steps = SIZEOF_ARRAY(grab);
            break;
        case CMD_DROP:
            step_rest.step_seq = &drop[0];
            step_rest.steps = SIZEOF_ARRAY(drop);
            break;
        case CMD_AUTO:
            is_auto = true;
            break;
        case CMD_SET_SPEED:
#ifdef DEBUG
            DEBUG_SERIAL.print("DBG: speed = ");
            DEBUG_SERIAL.println(speed);
#endif

            // Já que o valor corresponde a tempo decorrido, devemos inverter o
            // valor da velocidade para que seja diretamente proporcional
            if (speed != -1)
                ticks_per_step = (255 - speed) + MIN_TICKS_PER_STEP;

            break;
    }
}

void loop() {
#ifdef BLUETOOTH
    if (Serial2.available() >= 2) {
        int data_in = Serial2.read();
        int speed = Serial2.read();

        if (!is_auto) {

        } else {
            if (data_in == CMD_AUTO)
                is_auto = false;
        }

        // Inicia o timer
        if (!timer_has_run) {
            timer_has_run = true;
            time_last_step = time_last_meas = millis();
        }

        if (data_in == CMD_MOVE_FWD || data_in == CMD_MOVE_REV || 
            data_in == CMD_MOVE_ROT_LEFT || data_in == CMD_MOVE_ROT_RIGHT)
        {
            if (step_body.seq_dir == +1)
                step_body.cur_step = 0;
            else if (step_body.seq_dir == -1)
                step_body.cur_step = step_body.steps - 1;
        }
    }

    int32_t elapsed = millis();

    int32_t since_last_step = elapsed - time_last_step;

    if (since_last_step < 0)
        since_last_step = -since_last_step;

    int32_t since_last_meas = elapsed - time_last_meas;

    if (since_last_meas < 0)
        since_last_meas = -since_last_meas;

    // Esperar a compleção de um ciclo para retomar
    if (!timer_has_run || since_last_step < ticks_per_step || since_last_meas < SENSOR_REFRESH_INTERVAL)
        return;

    // XXX: ler informação dos sensores condicionalmente
    // XXX: filtrar aferições para reduzir ruído?
    if (since_last_meas > SENSOR_REFRESH_INTERVAL) {
#ifdef US_SENSOR
        if (since_last_meas < SENSOR_REFRESH_INTERVAL + 10) {
            if (!sent_meas_trig) {
                digitalWrite(US_TRIG_PIN, HIGH);
                sent_meas_trig = true;
            }
        } else {
#endif

#ifdef DEBUG
            DEBUG_SERIAL.print("DBG: Tempo decorrido desde a última aferição: ");
            DEBUG_SERIAL.print(since_last_meas);
            DEBUG_SERIAL.println("ms");
#endif

#ifdef US_SENSOR
            digitalWrite(US_TRIG_PIN, LOW);
            // XXX: calcular a distância e armazenar em sensor_data  
            uint32_t us_sensor_data = pulseIn(US_ECHO_PIN, HIGH);

            sensor_data.front = (us_sensor_data / 2) / 29;
#endif

#ifdef IR_SENSOR
            // XXX: converter para distância (v. monografia)
            sensor_data.left = sensor_left.readProximity();
            sensor_data.right = sensor_right.readProximity();
#endif

            sent_meas_trig = false;

#ifdef US_SENSOR            
        }
#endif
    }

    if (is_auto) {
        // ...

        // XXX: número mágico?
        if (sensor_data.front >= 7) {
            execute_command(CMD_MOVE_FWD, -1);
        } else if (sensor_data.left <= 5 && sensor_data.right <= 5 ) {
            execute_command(CMD_MOVE_REV, -1);

            // Virar para algum lado em seguida? ou continuar para trás?
        } else if (sensor_data.left < sensor_data.right) {
            execute_command(CMD_MOVE_ROT_LEFT, -1);
        } else {
            execute_command(CMD_MOVE_ROT_RIGHT, -1);
        }

        return;
    }

    if ((step_body.seq_dir == 1 && step_body.cur_step == step_body.steps) || (step_body.seq_dir == -1 && step_body.cur_step == -1))
        step_body.step_seq = nullptr;

    if (step_rest.cur_step == step_rest.steps - 1)
        step_rest.step_seq = nullptr;

    // Executar uma sequência de passos
    if (step_robot.step_seq != nullptr) {
        if (step_body.step_seq != nullptr)
            step_body.step_seq = nullptr;

        if (step_rest.step_seq != nullptr)
            step_rest.step_seq = nullptr;

        auto cur = step_robot.step_seq[step_robot.cur_step];

        for (int i = 0; i < 22; i++)
            servos[i].write(cur[i]);

        step_robot.cur_step++;
        time_last_step = millis();
    }

    if (step_body.step_seq != nullptr) {
        auto cur = step_body.step_seq[step_body.cur_step];
        auto dir = step_body.seq_dir;

        for (int i = 0; i < 18; i++)
            servos[servo_off::TORAX + i].write(cur[i]);

        step_body.cur_step += dir;
        time_last_step = millis();
    }

    if (step_rest.step_seq != nullptr) {
        auto cur = step_rest.step_seq[step_rest.cur_step];
        
        for (int i = 0; i < 5; i++) 
            servos[servo_off::REST + i].write(cur[i]);

        step_body.cur_step++;
        // XXX: Deduplicação de código?
        time_last_step = millis();
    }

#endif
}
