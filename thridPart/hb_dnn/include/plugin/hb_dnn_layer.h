// Copyright (c) 2024，D-Robotics.
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

// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_PLUGIN_HB_DNN_LAYER_H_
#define DNN_PLUGIN_HB_DNN_LAYER_H_

#include <string>
#include <vector>

#include "dnn/plugin/hb_dnn_ndarray.h"

namespace hobot {
namespace dnn {

class Attribute {
 public:
  virtual ~Attribute() = default;

  /**
   * Get integer value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(int32_t *val, char const *key) const = 0;

  /**
   * Get float value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(float *val, char const *key) const = 0;

  /**
   * Get string value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(std::string *val,
                                    char const *key) const = 0;
  /**
   * Get `NDArray` value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(NDArray *val, char const *key) const = 0;
  /**
   * Get float value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(std::vector<int32_t> *val,
                                    char const *key) const = 0;
  /**
   * Get vector float value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(std::vector<float> *val,
                                    char const *key) const = 0;
  /**
   * Get vector string value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(std::vector<std::string> *val,
                                    char const *key) const = 0;
  /**
   * Get vector `NDArray` value for a given key
   * @param[out] val: value of attributes
   * @param[in] key: key of attributes
   * @return 0 if success, return -1 otherwise
   */
  virtual int32_t GetAttributeValue(std::vector<NDArray> *val,
                                    char const *key) const = 0;
};  // class Attribute

struct hbDNNInferCtrlParam;
class Layer {
 public:
  Layer() = default;
  virtual ~Layer() = default;
  Layer(Layer const &) = delete;
  Layer &operator=(Layer const &) = delete;

  /**
   * Init layer
   * @param[in] attributes: attributes of the layer come from model file
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t Init(Attribute const &attributes) { return 0; }

  /**
   * Forward inference of the layer
   * @param[in] bottomBlobs: input blobs
   * @param[out] topBlobs: output blobs
   * @param[in] inferCtrlParam: infer control parameter
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t Forward(std::vector<NDArray *> const &bottomBlobs,
                          std::vector<NDArray *> &topBlobs,
                          hbDNNInferCtrlParam const *inferCtrlParam) = 0;

  /**
   * Get count of input blobs of layer
   * @return input count
   */
  virtual uint32_t GetInputCount() const { return 1U; }

  /**
   * Get count of output blobs of layer
   * @return output count
   */
  virtual uint32_t GetOutputCount() const { return 1U; }

  /**
   * Get layer type, such as Conv, Relu etc.
   * @return str of layer type
   */
  virtual std::string GetType() const = 0;
};  // class Layer

}  // namespace dnn
}  // namespace hobot

#endif  // DNN_PLUGIN_HB_DNN_LAYER_H_
