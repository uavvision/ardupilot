#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_VISUALCOURSE \
    LOG_VC_MSG

// @LoggerMessage: VC
// @Description: Visual course-hold (forward optical flow)
// @Field: TimeUS: Time since system startup
// @Field: Heal: True if visual course is healthy
// @Field: Crab: crab angle, ground velocity direction relative to nose
// @Field: CRate: crab rate
// @Field: Qual: last optical flow quality
// @Field: CErr: course error vs locked course
// @Field: RollCmd: commanded roll
// @Field: ATun: autotune active
// @Field: ATunP: autotune periods counted
// @Field: Sync: companion timestamps are in the FC timebase

struct PACKED log_VisualCourse {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    float crab;
    float crab_rate;
    uint8_t quality;
    float course_error;
    float roll_cmd;
    uint8_t atune_active;
    uint8_t atune_periods;
    uint8_t synced;
};

#define LOG_STRUCTURE_FROM_VISUALCOURSE \
    { LOG_VC_MSG, sizeof(log_VisualCourse), \
      "VC", "QBffBffBBB", "TimeUS,Heal,Crab,CRate,Qual,CErr,RollCmd,ATun,ATunP,Sync", "s-rE-rr---", "F-00-00---" , true },
