// Copyright 2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#define WORLD_CONTROL_BASE                                      0x600D0000
#define WORLD_CONTROL_CORE_X_BASE(i)                            ((WORLD_CONTROL_BASE) + 0x400*(i))

#define WORLD_CONTROL_CORE_X_MTVEC_BASE(i)                      (WORLD_CONTROL_CORE_X_BASE(i) + 0x000)
#define WORLD_CONTROL_CORE_X_MSTATUS_MIE(i)                     (WORLD_CONTROL_CORE_X_BASE(i) + 0x004)
#define WORLD_CONTROL_CORE_X_ENTRY_CHECK(i)                     (WORLD_CONTROL_CORE_X_BASE(i) + 0x008)
#define WORLD_CONTROL_CORE_X_WORLD_CAHNGE_OPT(i)                (WORLD_CONTROL_CORE_X_BASE(i) + 0x020)
#define WORLD_CONTROL_CORE_X_MESSAGE_ADDR(i)                    (WORLD_CONTROL_CORE_X_BASE(i) + 0x024)
#define WORLD_CONTROL_CORE_X_MESSAGE_MAX(i)                     (WORLD_CONTROL_CORE_X_BASE(i) + 0x028)
#define WORLD_CONTROL_CORE_X_STATUSTABLE(i, j)                  (WORLD_CONTROL_CORE_X_BASE(i) + 0x040 + 0x4*(j))
#define WORLD_CONTROL_CORE_X_STATUSTABLE_CURRENT(i)             (WORLD_CONTROL_CORE_X_BASE(i) + 0x0E0)
#define WORLD_CONTROL_CORE_X_MESSAGE_PHASE(i)                   (WORLD_CONTROL_CORE_X_BASE(i) + 0x0E4)
#define WORLD_CONTROL_CORE_X_WORLD_TRIGGER_ADDR(i)              (WORLD_CONTROL_CORE_X_BASE(i) + 0x140)
#define WORLD_CONTROL_CORE_X_WORLD_PREPARE(i)                   (WORLD_CONTROL_CORE_X_BASE(i) + 0x144)
#define WORLD_CONTROL_CORE_X_WORLD_UPDATE(i)                    (WORLD_CONTROL_CORE_X_BASE(i) + 0x148)
#define WORLD_CONTROL_CORE_X_WORLD_CANCEL(i)                    (WORLD_CONTROL_CORE_X_BASE(i) + 0x14c)
#define WORLD_CONTROL_CORE_X_WORLD_IRAM0(i)                     (WORLD_CONTROL_CORE_X_BASE(i) + 0x150)
#define WORLD_CONTROL_CORE_X_WORLD_DRAM0_PIF(i)                 (WORLD_CONTROL_CORE_X_BASE(i) + 0x154)
#define WORLD_CONTROL_CORE_X_WORLD_PHASE(i)                     (WORLD_CONTROL_CORE_X_BASE(i) + 0x158)
#define WORLD_CONTROL_CORE_X_NMI_MASK_ENABLE(i)                 (WORLD_CONTROL_CORE_X_BASE(i) + 0x180)
#define WORLD_CONTROL_CORE_X_NMI_MASK_TRIGGER_ADDR(i)           (WORLD_CONTROL_CORE_X_BASE(i) + 0x184)
#define WORLD_CONTROL_CORE_X_NMI_MASK_DISABLE(i)                (WORLD_CONTROL_CORE_X_BASE(i) + 0x188)
#define WORLD_CONTROL_CORE_X_NMI_MASK_CANCEL(i)                 (WORLD_CONTROL_CORE_X_BASE(i) + 0x18c)
#define WORLD_CONTROL_CORE_X_NMI_MASK(i)                        (WORLD_CONTROL_CORE_X_BASE(i) + 0x190)
#define WORLD_CONTROL_CORE_X_NMI_MASK_PHASE(i)                  (WORLD_CONTROL_CORE_X_BASE(i) + 0x194)

#define WORLD_CONTROLLER_DMA_APBPERI_SPI2                       (WORLD_CONTROL_BASE + 0x800)
#define WORLD_CONTROLLER_DMA_APBPERI_SPI3                       (WORLD_CONTROL_BASE + 0x804)
#define WORLD_CONTROLLER_DMA_APBPERI_UCHI0                      (WORLD_CONTROL_BASE + 0x808)
#define WORLD_CONTROLLER_DMA_APBPERI_I2S0                       (WORLD_CONTROL_BASE + 0x80C)
#define WORLD_CONTROLLER_DMA_APBPERI_I2S1                       (WORLD_CONTROL_BASE + 0x810)
#define WORLD_CONTROLLER_DMA_APBPERI_LCD_CAM                    (WORLD_CONTROL_BASE + 0x814)
#define WORLD_CONTROLLER_DMA_APBPERI_AES                        (WORLD_CONTROL_BASE + 0x818)
#define WORLD_CONTROLLER_DMA_APBPERI_SHA                        (WORLD_CONTROL_BASE + 0x81C)
#define WORLD_CONTROLLER_DMA_APBPERI_ADC_DAC                    (WORLD_CONTROL_BASE + 0x820)
#define WORLD_CONTROLLER_DMA_APBPERI_USB                        (WORLD_CONTROL_BASE + 0x824)
#define WORLD_CONTROLLER_DMA_APBPERI_SDIO_HOST                  (WORLD_CONTROL_BASE + 0x828)
#define WORLD_CONTROLLER_DMA_APBPERI_MAC                        (WORLD_CONTROL_BASE + 0x82C)
#define WORLD_CONTROLLER_DMA_APBPERI_BACKUP                     (WORLD_CONTROL_BASE + 0x830)
#define WORLD_CONTROLLER_DMA_APBPERI_SLC                        (WORLD_CONTROL_BASE + 0x834)
#define WORLD_CONTROLLER_DMA_APBPERI_LC                         (WORLD_CONTROL_BASE + 0x838)
#define WORLD_CONTROLLER_EDMA_SPI2                              (WORLD_CONTROL_BASE + 0x900)
#define WORLD_CONTROLLER_EDMA_SPI3                              (WORLD_CONTROL_BASE + 0x904)
#define WORLD_CONTROLLER_EDMA_UCHI0                             (WORLD_CONTROL_BASE + 0x908)
#define WORLD_CONTROLLER_EDMA_I2S0                              (WORLD_CONTROL_BASE + 0x90C)
#define WORLD_CONTROLLER_EDMA_I2S1                              (WORLD_CONTROL_BASE + 0x910)
#define WORLD_CONTROLLER_EDMA_LCD_CAM                           (WORLD_CONTROL_BASE + 0x914)
#define WORLD_CONTROLLER_EDMA_AES                               (WORLD_CONTROL_BASE + 0x918)
#define WORLD_CONTROLLER_EDMA_SHA                               (WORLD_CONTROL_BASE + 0x91C)
#define WORLD_CONTROLLER_EDMA_ADC_DAC                           (WORLD_CONTROL_BASE + 0x920)
