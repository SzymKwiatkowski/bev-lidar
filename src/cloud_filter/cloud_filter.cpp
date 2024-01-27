// Copyright 2024 Szymon
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bev_lidar/cloud_filter/cloud_filter.hpp"

#include <iostream>

namespace cloud_filter
{

CloudFilter::CloudFilter()
{
}

int64_t CloudFilter::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

}  // namespace bev_lidar
