// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "netdev_api.h"

/** Exported macros **/
#define ARPING_DEMO	1
#define LWIP_DEMO	2
#define MAIN_APP_CODE LWIP_DEMO

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/
void hosted_initialize(void);
extern stm_ret_t send_arp_req(struct network_handle *net_handle, uint8_t *src_mac,
		uint32_t *src_ip, uint8_t *dst_mac, uint32_t *dst_ip);

#ifdef __cplusplus
}
#endif

#endif
