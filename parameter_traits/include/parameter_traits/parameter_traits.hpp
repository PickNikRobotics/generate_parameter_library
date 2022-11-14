// Copyright (c) 2022, PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <fmt/core.h>
#include <rsl/algorithm.hpp>
#include <tl_expected/expected.hpp>

namespace parameter_traits {

using Result
    [[deprecated("Use tl::expected<void, std::string> for return instead. "
                 "`#include <tl_expected/expected.hpp>`.")]] =
        tl::expected<void, std::string>;

template <typename... Args>
[[deprecated(
    "When returning tl::expected<void, std::string> you can call fmt::format "
    "directly.")]] auto
ERROR(const std::string& format, Args... args)
    -> tl::expected<void, std::string> {
  return tl::make_unexpected(fmt::format(format, args...));
}

auto static OK
    [[deprecated("When returning tl::expected<void, std::string> default "
                 "construct for OK with `{}`.")]] =
        tl::expected<void, std::string>{};

template <typename T>
[[deprecated("Use rsl::contains instead. `#include <rsl/algorithm.hpp>`")]] bool
contains(std::vector<T> const& vec, T const& val) {
  return rsl::contains(vec, val);
}

template <class T>
[[deprecated(
    "Use rsl::is_unique instead. `#include <rsl/algorithm.hpp>`")]] bool
is_unique(std::vector<T> const& x) {
  return rsl::is_unique(x);
}

}  // namespace parameter_traits
