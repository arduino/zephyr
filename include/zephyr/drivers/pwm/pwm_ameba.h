/* The custom flags are on the upper 8bits of the pwm_flags_t
 * @{
 */
/* PWM pin polarity */
/* 0 -> active-high pulse  1 -> active-low pulse*/
#define AMEBA_PWM_POLARITY          (1U << 0)
/* PWM configuration flags: bit8 is used for config OCPROTECTION*/
/* 0 -> ENABLE OCPROTECTION function  1 -> DISABLE OCPROTECTION function*/
#define AMEBA_PWM_OCPROTECTION      (1U << 8)
/* PWM configuration flags: bit9 is used for mode select*/
/* 0 -> pwm mode  1 -> one-pulse mode*/
#define AMEBA_PWM_MODE              (1U << 9)
/* One-pulse mode configuration flags: bit10 and bit11 is used for external trigger polarity*/
/* 0 -> positive edge active  1 -> negative edge active  2 or 3 -> both edge active*/
#define AMEBA_OPMode_ETP_MASK       (3U << 10)
#define AMEBA_OPMode_ETP_ActiveEdge (1U << 10)
#define AMEBA_OPMode_ETP_BothActive (1U << 11)
/* One-pulse mode configuration flags: bit12 is used for One-pulse mode default level*/
/* 0 -> default level is 0   1 -> default level is 1*/
#define AMEBA_OPMode_DefaultLevel   (1U << 12)
