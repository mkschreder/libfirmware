/** :ms-top-comment
*                       ____                  _ _     _
*                      / ___|_      _____  __| (_)___| |__
*                      \___ \ \ /\ / / _ \/ _` | / __| '_ \
*                       ___) \ V  V /  __/ (_| | \__ \ | | |
*                      |____/ \_/\_/ \___|\__,_|_|___/_| |_|
*
*                _____           _              _     _          _
*               | ____|_ __ ___ | |__   ___  __| | __| | ___  __| |
*               |  _| | '_ ` _ \| '_ \ / _ \/ _` |/ _` |/ _ \/ _` |
*               | |___| | | | | | |_) |  __/ (_| | (_| |  __/ (_| |
*               |_____|_| |_| |_|_.__/ \___|\__,_|\__,_|\___|\__,_|
*
*
* This file is part of TheBoss Project
*
* @file include/libfirmware/pm.h
* @author Martin K. Schröder
* @version
* @date 2019-05-25
* @homepage https://github.com/mkschreder/libfirmware
*
* Copyright (C) 2014-2019 Martin K. Schröder. All rights reserved.
* Author: Martin K. Schröder <mkschreder.uk@gmail.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name NuttX nor the names of its contributors may be used to
* endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#pragma once

#include "device.h"

typedef const struct pm_device_ops **pm_device_t;

struct pm_device_ops {
	void (*wake_up)(pm_device_t dev);
	void (*suspend)(pm_device_t dev);
};

#define pm_suspend(dev)             \
	dev_check_call(dev, suspend)
#define pm_wake_up(dev)             \
	dev_check_call(dev, wake_up)
