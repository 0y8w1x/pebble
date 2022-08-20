/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/io/dataset_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/optical_flow/optical_flow.h>

namespace cereal {

template <class Archive, class T>
void save(Archive& ar, const basalt::ManagedImage<T>& m) {
  ar(m.w);
  ar(m.h);
  ar(cereal::binary_data(m.ptr, sizeof(T) * m.w * m.h));
}

template <class Archive, class T>
void load(Archive& ar, basalt::ManagedImage<T>& m) {
  size_t w;
  size_t h;
  ar(w);
  ar(h);
  m.Reinitialise(w, h);
  ar(cereal::binary_data(m.ptr, sizeof(T) * m.w * m.h));
}

template <class Archive>
void serialize(Archive& ar, basalt::OpticalFlowResult& m) {
  ar(m.t_ns);
  ar(m.observations);
  ar(m.input_images);
}

template <class Archive>
void serialize(Archive& ar, basalt::OpticalFlowInput& m) {
  ar(m.t_ns);
  ar(m.img_data);
}

template <class Archive>
void serialize(Archive& ar, basalt::ImageData& m) {
  ar(m.exposure);
  ar(m.img);
}

template <class Archive>
static void serialize(Archive& ar, Eigen::AffineCompact2f& m) {
  ar(m.matrix());
}
}  // namespace cereal
