/**
***************************************************************
* @file apps/server/include/prov.h
* @author Lachlan Smith - s4482220
* @date 08032021
* @brief The Methane Project
*************************************************************** 
*/

#ifndef BT_MESH_ARGON_PROV_H__
#define BT_MESH_ARGON_PROV_H__

#include <bluetooth/mesh.h>

/** @brief Initialize the provisioning handler for the argon.
 *
 * @return The provisioning properties to pass to @em bt_mesh_init().
 */
const struct bt_mesh_prov *bt_mesh_argon_prov_init(void);

#endif /* BT_MESH_ARGON_PROV_H__ */
