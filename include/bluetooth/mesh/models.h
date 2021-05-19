/*
 * Copyright (c) 2019 - 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_MESH_MODELS_H__
#define BT_MESH_MODELS_H__

#include <bluetooth/mesh.h>

#include "model_types.h"

/* Foundation models */
#include <bluetooth/mesh/cfg_cli.h>
#include <bluetooth/mesh/cfg_srv.h>
#include <bluetooth/mesh/health_cli.h>
#include <bluetooth/mesh/health_srv.h>

/* Sensor models */
#include <bluetooth/mesh/sensor_types.h>
#include <bluetooth/mesh/sensor_srv.h>
#include <bluetooth/mesh/sensor_cli.h>

/** @brief Check whether the model publishes to a unicast address.
 *
 * @param[in] model Model to check
 *
 * @return true if the model publishes to a unicast address, false otherwise.
 */
bool bt_mesh_model_pub_is_unicast(const struct bt_mesh_model *model);

/** Shorthand macro for defining a model list directly in the element. */
#define BT_MESH_MODEL_LIST(...) ((struct bt_mesh_model[]){ __VA_ARGS__ })

#endif /* BT_MESH_MODELS_H__ */
