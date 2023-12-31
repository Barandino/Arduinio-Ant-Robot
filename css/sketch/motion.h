#ifndef MOTION_H
#define MOTION_H

#include <cstdint>

typedef uint8_t step_list_body_t[18];
typedef uint8_t step_list_rest_t[4];

typedef uint8_t step_list_robot_t[22];

struct step_body_t {
    const step_list_body_t *step_seq;
    uint8_t steps;
    uint8_t cur_step;
    uint8_t seq_dir;
};

struct step_rest_t {
    const step_list_rest_t *step_seq;
    uint8_t steps;
    uint8_t cur_step;
};

struct step_robot_t {
    const step_list_robot_t *step_seq;
    uint8_t steps;
    uint8_t cur_step;
};

static const step_list_body_t forward[] = {
    { 80, 60, 90, 100, 90, 90,  80, 60, 90,  80, 90, 90, 100, 60, 90,  80, 90, 90},
    { 80, 90, 90, 100, 90, 90,  80, 90, 90,  80, 90, 90, 100, 90, 90,  80, 90, 90},
    { 80, 90, 90, 100, 60, 90,  80, 90, 90,  80, 60, 90, 100, 90, 90,  80, 60, 90},
    {100, 90, 90,  80, 60, 90, 100, 90, 90, 100, 60, 90,  80, 90, 90, 100, 60, 90},
    {100, 90, 90,  80, 90, 90, 100, 90, 90, 100, 90, 90,  80, 90, 90, 100, 90, 90},
    {100, 60, 90,  80, 90, 90, 100, 60, 90, 100, 90, 90,  80, 60, 90, 100, 90, 90}
};

static const step_list_body_t rotate_cw[] = {
    { 80, 60, 90, 100, 90, 90,  80, 60, 90, 100, 90, 90,  80, 60, 90, 100, 90, 90},
    { 80, 90, 90, 100, 90, 90,  80, 90, 90, 100, 90, 90,  80, 90, 90, 100, 90, 90},
    { 80, 90, 90, 100, 60, 90,  80, 90, 90, 100, 60, 90,  80, 90, 90, 100, 60, 90},
    {100, 90, 90,  80, 60, 90, 100, 90, 90,  80, 60, 90, 100, 90, 90,  80, 60, 90},
    {100, 90, 90,  80, 90, 90, 100, 90, 90,  80, 90, 90, 100, 90, 90,  80, 90, 90},
    {100, 60, 90,  80, 90, 90, 100, 60, 90,  80, 90, 90, 100, 60, 90,  80, 90, 90}
};

// XXX: Usar -1 em vez de 90 para compor movimentos de vários comandos ao mesmo tempo?
static const step_list_rest_t bite[] = {
    {90, 90, 90, 90},
    {90, 90,  0, 90}
  /* ... */
};

static const step_list_robot_t attack[] = {
    {90, 110, 90, 90,  80, 90, 90,  80, 90, 90,  80, 90, 90,  80, 90, 90,  80, 90, 90,  80, 90, 90},
    {90, 110, 90, 90, 110, 90, 90, 110, 90, 90, 110, 90, 90, 110, 90, 90, 110, 90, 90, 110, 90, 90}
};

static const step_list_rest_t grab[] = {
    {90, 90, 50, 90}
};

static const step_list_rest_t drop[] = {
    {90, 90, 150, 90}
};

#endif
