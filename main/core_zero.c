#include "core_zero.h"
#include "servo_control.h"

/* La capa ROS 2 se compila junto al componente micro-ROS Jazzy del proyecto. */
#ifdef ESP_CAID_ENABLE_MICROROS

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "uros_network_interfaces.h"

#include "geometry_msgs/msg/twist.h"
#include "sensor_msgs/msg/joint_state.h"
#include "sensor_msgs/msg/range.h"
#include "std_msgs/msg/int16.h"
#include "micro_ros_utilities/type_utilities.h"
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"

/* IP y puerto del agent: idf.py menuconfig -> micro-ROS Settings. */

#define TAG "comms"

#define RAD_TO_DEG       57.2957795f
#define ARM_CENTER_DEG   90.0f      /* /arm_pos en radianes, 0 rad = 90 grados de servo */
#define US_MIN_RANGE_M   0.02f
#define US_MAX_RANGE_M   4.0f
#define US_FOV_RAD       0.26f
#define AGENT_PING_MS    1000

#define RCCHECK(fn) do { rcl_ret_t rc_ = (fn); if (rc_ != RCL_RET_OK) { \
    ESP_LOGE(TAG, "%s:%d fallo rc=%d", __FILE__, __LINE__, (int)rc_); return false; } } while (0)
#define RCSOFT(fn) do { rcl_ret_t rc_ = (fn); if (rc_ != RCL_RET_OK) { \
    ESP_LOGW(TAG, "%s:%d rc=%d", __FILE__, __LINE__, (int)rc_); } } while (0)

/* ---- Entidades ROS ---- */
static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rclc_executor_t g_executor;
static rcl_timer_t g_timer_telemetria;
static rcl_subscription_t g_sub_cmd_vel, g_sub_cam, g_sub_arm;
static rcl_publisher_t g_pub_ticks, g_pub_front, g_pub_left, g_pub_right;
static volatile bool g_ros_ready = false;

/* ---- Mensajes de entrada ---- */
static geometry_msgs__msg__Twist g_msg_twist;
static std_msgs__msg__Int16 g_msg_cam;
static sensor_msgs__msg__JointState g_msg_arm;   /* memoria reservada con micro_ros_utilities */

/* ---- Mensajes de salida (estáticos, sin heap) ---- */
static sensor_msgs__msg__JointState g_msg_ticks;
static double g_ticks_pos[2];
static char g_ticks_name_l[] = "left_wheel";
static char g_ticks_name_r[] = "right_wheel";
static rosidl_runtime_c__String g_ticks_names[2];

static sensor_msgs__msg__Range g_msg_range[3];
static char g_range_frame[3][16];

static void init_mensajes(void) {
    /* Entrada: JointState necesita capacidad reservada antes de deserializar. */
    micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 16;
    conf.max_ros2_type_sequence_capacity = 8;
    conf.max_basic_type_sequence_capacity = 8;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &g_msg_arm, conf)) {
        ESP_LOGE(TAG, "sin memoria para JointState");
        vTaskDelete(NULL);
    }

    /* /wheel_ticks: JointState con position = ticks acumulados [izq, der]. */
    g_ticks_names[0].data = g_ticks_name_l;
    g_ticks_names[0].size = sizeof(g_ticks_name_l) - 1;
    g_ticks_names[0].capacity = sizeof(g_ticks_name_l);
    g_ticks_names[1].data = g_ticks_name_r;
    g_ticks_names[1].size = sizeof(g_ticks_name_r) - 1;
    g_ticks_names[1].capacity = sizeof(g_ticks_name_r);
    g_msg_ticks.name.data = g_ticks_names;
    g_msg_ticks.name.size = 2;
    g_msg_ticks.name.capacity = 2;
    g_msg_ticks.position.data = g_ticks_pos;
    g_msg_ticks.position.size = 2;
    g_msg_ticks.position.capacity = 2;

    static const char *frames[3] = {"us_frente", "us_izquierda", "us_derecha"};
    for (int i = 0; i < 3; ++i) {
        snprintf(g_range_frame[i], sizeof(g_range_frame[i]), "%s", frames[i]);
        g_msg_range[i].header.frame_id.data = g_range_frame[i];
        g_msg_range[i].header.frame_id.size = strlen(g_range_frame[i]);
        g_msg_range[i].header.frame_id.capacity = sizeof(g_range_frame[i]);
        g_msg_range[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        g_msg_range[i].field_of_view = US_FOV_RAD;
        g_msg_range[i].min_range = US_MIN_RANGE_M;
        g_msg_range[i].max_range = US_MAX_RANGE_M;
    }
}

/* ---- Callbacks: solo guardan el objetivo y salen (nada bloqueante) ---- */
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    if (!isfinite(msg->linear.x) || !isfinite(msg->angular.z)) {
        return;
    }
    setpoint_escribir((float)msg->linear.x, (float)msg->angular.z);
}

static void cam_callback(const void *msgin) {
    const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
    servo_ctrl_set_camera_target((float)msg->data);   /* grados 0..180 */
}

static void arm_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
    if (msg->position.size == 0) {
        return;
    }
    float deg[BRAZO_NUM_ARTICULACIONES] = {0};
    const size_t count = msg->position.size < BRAZO_NUM_ARTICULACIONES
                             ? msg->position.size
                             : BRAZO_NUM_ARTICULACIONES;
    for (size_t i = 0; i < count; ++i) {
        deg[i] = ARM_CENTER_DEG + (float)msg->position.data[i] * RAD_TO_DEG;
    }
    servo_ctrl_set_arm_target(deg, count);
}

void iniciar_wifi(void) {
    if (uros_network_interface_initialize() != ESP_OK) {
        ESP_LOGE(TAG, "fallo la interfaz de red");
        vTaskDelete(NULL);
    }
}

void publicar_telemetria(const telemetria_t *snapshot) {
    if (!g_ros_ready || snapshot == NULL) {
        return;
    }

    g_ticks_pos[0] = (double)snapshot->ticks_izq;
    g_ticks_pos[1] = (double)snapshot->ticks_der;
    RCSOFT(rcl_publish(&g_pub_ticks, &g_msg_ticks, NULL));

    const float dist[3] = {snapshot->dist_frente_m, snapshot->dist_izq_m, snapshot->dist_der_m};
    rcl_publisher_t *pubs[3] = {&g_pub_front, &g_pub_left, &g_pub_right};
    for (int i = 0; i < 3; ++i) {
        /* REP-117: sin obstáculo / fuera de rango se publica +inf. */
        g_msg_range[i].range = (isfinite(dist[i]) && dist[i] <= US_MAX_RANGE_M) ? dist[i] : INFINITY;
        RCSOFT(rcl_publish(pubs[i], &g_msg_range[i], NULL));
    }
}

static void timer_telemetria_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    telemetria_t snapshot = {0};
    if (xQueueReceive(cola_telemetria, &snapshot, 0) == pdPASS) {
        publicar_telemetria(&snapshot);
    }
}

static bool create_entities(void) {
    g_allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, g_allocator));
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT,
                                             rcl_init_options_get_rmw_init_options(&init_options)));
    RCCHECK(rclc_support_init_with_options(&g_support, 0, NULL, &init_options, &g_allocator));
    
    RCCHECK(rclc_node_init_default(&g_node, "esp32_caid_node", "", &g_support));

    RCCHECK(
        rclc_subscription_init_default(
            &g_sub_cmd_vel, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
            "/cmd_vel"
        )
    );
    
    RCCHECK(
        rclc_subscription_init_default(
            &g_sub_cam, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
            "/cam_angle"
        )
    );
    
    RCCHECK(
        rclc_subscription_init_default(
            &g_sub_arm, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
            "/arm_pos"
        )
    );

    RCCHECK(
        rclc_publisher_init_default(
            &g_pub_ticks, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
            "/wheel_ticks"
        )
    );

    RCCHECK(
        rclc_publisher_init_default(
            &g_pub_front, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), 
            "/ultrasonicos/frente"
        )
    );

    RCCHECK(
        rclc_publisher_init_default(
            &g_pub_left, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), 
            "/ultrasonicos/izquierda"
        )
    );

    RCCHECK(
        rclc_publisher_init_default(
            &g_pub_right, 
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), 
            "/ultrasonicos/derecha"
        )
    );

    RCCHECK(rclc_timer_init_default2(&g_timer_telemetria, &g_support,
            RCL_MS_TO_NS(PERIODO_TELEMETRIA_MS), timer_telemetria_callback, true));

    RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 4, &g_allocator));

    RCCHECK(
        rclc_executor_add_subscription(
            &g_executor, 
            &g_sub_cmd_vel, 
            &g_msg_twist,
            &cmd_vel_callback, 
            ON_NEW_DATA
        )
    );

    RCCHECK(
        rclc_executor_add_subscription(
            &g_executor, 
            &g_sub_cam, 
            &g_msg_cam,
            &cam_callback, 
            ON_NEW_DATA
        )
    );

    RCCHECK(
        rclc_executor_add_subscription(
            &g_executor, 
            &g_sub_arm, 
            &g_msg_arm,
            &arm_callback, 
            ON_NEW_DATA
        )
    );

    RCCHECK(rclc_executor_add_timer(&g_executor, &g_timer_telemetria));
    return true;
}

static void destroy_entities(void) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&g_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rcl_publisher_fini(&g_pub_ticks, &g_node);
    (void)rcl_publisher_fini(&g_pub_front, &g_node);
    (void)rcl_publisher_fini(&g_pub_left, &g_node);
    (void)rcl_publisher_fini(&g_pub_right, &g_node);
    (void)rcl_subscription_fini(&g_sub_cmd_vel, &g_node);
    (void)rcl_subscription_fini(&g_sub_cam, &g_node);
    (void)rcl_subscription_fini(&g_sub_arm, &g_node);
    (void)rcl_timer_fini(&g_timer_telemetria);
    (void)rclc_executor_fini(&g_executor);
    (void)rcl_node_fini(&g_node);
    (void)rclc_support_fini(&g_support);
}

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } comms_state_t;

void vTaskCore0_MicroROS(void *pvParameters) {
    (void)pvParameters;

    iniciar_wifi();
    init_mensajes();

    comms_state_t state = WAITING_AGENT;
    int64_t last_ping_us = 0;

    while (1) {
        switch (state) {
        case WAITING_AGENT:
            state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                g_ros_ready = true;
                last_ping_us = esp_timer_get_time();
                state = AGENT_CONNECTED;
                ESP_LOGI(TAG, "conectado al agent");
            } else {
                destroy_entities();
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            if ((esp_timer_get_time() - last_ping_us) > AGENT_PING_MS * 1000LL) {
                last_ping_us = esp_timer_get_time();
                if (rmw_uros_ping_agent(100, 2) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10));
            vTaskDelay(pdMS_TO_TICKS(5));
            break;

        case AGENT_DISCONNECTED:
            ESP_LOGW(TAG, "agent perdido, reconectando");
            g_ros_ready = false;
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }
}

#else

void iniciar_wifi(void) {
    /* Sin micro-ROS no se inicializa la conexión de comunicaciones. */
}

void publicar_telemetria(const telemetria_t *snapshot) {
    (void)snapshot;
}

void vTaskCore0_MicroROS(void *pvParameters) {
    (void)pvParameters;
    vTaskDelete(NULL);
}

#endif