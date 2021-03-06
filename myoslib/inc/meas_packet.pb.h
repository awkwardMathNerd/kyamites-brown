/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.6-dev */

#ifndef PB_MEAS_PACKET_PB_H_INCLUDED
#define PB_MEAS_PACKET_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _meas_packet_message { 
    uint32_t methane_concentration; 
    float calibration_value; 
} meas_packet_message;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define meas_packet_message_init_default         {0, 0}
#define meas_packet_message_init_zero            {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define meas_packet_message_methane_concentration_tag 1
#define meas_packet_message_calibration_value_tag 2

/* Struct field encoding specification for nanopb */
#define meas_packet_message_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   methane_concentration,   1) \
X(a, STATIC,   REQUIRED, FLOAT,    calibration_value,   2)
#define meas_packet_message_CALLBACK NULL
#define meas_packet_message_DEFAULT NULL

extern const pb_msgdesc_t meas_packet_message_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define meas_packet_message_fields &meas_packet_message_msg

/* Maximum encoded size of messages (where known) */
#define meas_packet_message_size                 11

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
