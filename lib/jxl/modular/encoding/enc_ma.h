// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_MODULAR_ENCODING_ENC_MA_H_
#define LIB_JXL_MODULAR_ENCODING_ENC_MA_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/base/common.h"
#include "lib/jxl/base/random.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/enc_ans.h"
#include "lib/jxl/modular/encoding/context_predict.h"
#include "lib/jxl/modular/encoding/dec_ma.h"
#include "lib/jxl/modular/modular_image.h"
#include "lib/jxl/modular/options.h"

namespace jxl {

struct ResidualToken {
  uint8_t tok;
  uint8_t nbits;
};

// Auxiliary enum to define what data should be collected at sampling
enum class SampleMask : uint32_t {
  nothing = 0,
  channel = 1 << 0,   // channel_pixel_count
  group_id = 1 << 1,  // group_pixel_count
  coordinate = 1 << 2,
  pixel = 1 << 3,      // pixel_samples
  abs_pixel = 1 << 4,  // pixel_samples
  diff = 1 << 5,       // diff_samples
  abs_diff = 1 << 6,   // diff_samples
  wp = 1 << 7,

  need_sampling = channel | group_id | pixel | abs_pixel | diff | abs_diff
};
inline SampleMask& operator|=(SampleMask& lhs, const SampleMask& rhs) {
  using ut = std::underlying_type<SampleMask>::type;
  return lhs = SampleMask(ut(lhs) | ut(rhs));
}
inline SampleMask operator&(const SampleMask& lhs, const SampleMask& rhs) {
  using ut = std::underlying_type<SampleMask>::type;
  return SampleMask(ut(lhs) & ut(rhs));
}
inline SampleMask operator|(const SampleMask& lhs, const SampleMask& rhs) {
  using ut = std::underlying_type<SampleMask>::type;
  return SampleMask(ut(lhs) | ut(rhs));
}

inline SampleMask FromProperties(const std::vector<uint32_t>& props_to_use) {
  SampleMask sample_mask = SampleMask::nothing;
  const size_t props_to_use_size = props_to_use.size();

  for (size_t i = 0; i < props_to_use_size; i++) {
    JXL_DASSERT(props_to_use[i] < kNumNonrefProperties);

    if (props_to_use[i] == 0) {
      sample_mask |= SampleMask::channel;
    } else if (props_to_use[i] == 1) {
      sample_mask |= SampleMask::group_id;
    } else if (props_to_use[i] == 2 || props_to_use[i] == 3) {
      sample_mask |= SampleMask::coordinate;
    } else if (props_to_use[i] == 6 || props_to_use[i] == 7 ||
               props_to_use[i] == 8 ||
               (props_to_use[i] >= kNumNonrefProperties &&
                (props_to_use[i] - kNumNonrefProperties) % 4 == 1)) {
      sample_mask |= SampleMask::pixel;
    } else if (props_to_use[i] == 4 || props_to_use[i] == 5 ||
               (props_to_use[i] >= kNumNonrefProperties &&
                (props_to_use[i] - kNumNonrefProperties) % 4 == 0)) {
      sample_mask |= SampleMask::abs_pixel;
    } else if (props_to_use[i] >= kNumNonrefProperties &&
               (props_to_use[i] - kNumNonrefProperties) % 4 == 2) {
      sample_mask |= SampleMask::abs_diff;
    } else if (props_to_use[i] == kWPProp) {
      sample_mask |= SampleMask::wp;
    } else {
      sample_mask |= SampleMask::diff;
    }
  }

  return sample_mask;
}

// Struct to collect all the data needed to build a tree.
struct TreeSamples {
  static StatusOr<TreeSamples> Create(const ModularOptions &options) {
    TreeSamples tree_samples;
    JXL_RETURN_IF_ERROR(tree_samples.SetPredictor(options.predictor,
                                                options.wp_tree_mode));
    JXL_RETURN_IF_ERROR(
      tree_samples.SetProperties(options.splitting_heuristics_properties,
                                 options.wp_tree_mode));
    return tree_samples;
  }

  bool HasSamples() const {
    return !residuals.empty() && !residuals[0].empty();
  }
  size_t NumDistinctSamples() const { return sample_counts.size(); }
  size_t NumSamples() const { return num_samples; }
  uint32_t NumChannels() const { return samples.channel_pixel_count.size(); }

  const std::vector<ResidualToken>& RTokens(size_t pred) const {
    return residuals[pred];
  }
  const ResidualToken &RToken(size_t pred, size_t i) const {
    return residuals[pred][i];
  }
  size_t Token(size_t pred, size_t i) const { return residuals[pred][i].tok; }
  size_t Count(size_t i) const { return sample_counts[i]; }
  size_t PredictorIndex(Predictor predictor) const {
    const auto predictor_elem =
        std::find(predictors.begin(), predictors.end(), predictor);
    JXL_DASSERT(predictor_elem != predictors.end());
    return predictor_elem - predictors.begin();
  }
  size_t PropertyIndex(size_t property) const {
    const auto property_elem =
        std::find(props_to_use.begin(), props_to_use.end(), property);
    JXL_DASSERT(property_elem != props_to_use.end());
    return property_elem - props_to_use.begin();
  }
  size_t NumPropertyValues(size_t property_index) const {
    return compact_properties[property_index].size() + 1;
  }
  size_t NumStaticProps() const { return num_static_props; }
  // Returns the *quantized* property value.
  template<bool S>
  size_t Property(size_t property_index, size_t i) const {
    if (S) {
      return static_props[property_index][i];
    } else {
      return props[property_index][i];
    }
  }
  int UnquantizeProperty(size_t property_index, uint32_t quant) const {
    JXL_DASSERT(quant < compact_properties[property_index].size());
    return compact_properties[property_index][quant];
  }

  Predictor PredictorFromIndex(size_t index) const {
    JXL_DASSERT(index < predictors.size());
    return predictors[index];
  }
  size_t PropertyFromIndex(size_t index) const {
    JXL_DASSERT(index < props_to_use.size());
    return props_to_use[index];
  }
  size_t NumPredictors() const { return predictors.size(); }
  size_t NumProperties() const { return props_to_use.size(); }

  // Preallocate data for a given number of samples. MUST be called before
  // adding any sample.
  void PrepareForSamples(size_t extra_num_samples);
  // Add a sample.
  void AddSample(pixel_type_w pixel, const Properties &properties,
                 const pixel_type_w *predictions);
  // Pre-cluster property values.
  void PreQuantizeProperties(
      const StaticPropRange &range,
      const std::vector<ModularMultiplierInfo> &multiplier_info,
      size_t max_property_values);

  void AllSamplesDone() { dedup_table_ = std::vector<uint32_t>(); }

  uint32_t QuantizeProperty(uint32_t prop, pixel_type v) const {
    JXL_DASSERT(prop >= num_static_props);
    v = jxl::Clamp1(v, -kPropertyRange, kPropertyRange) + kPropertyRange;
    return property_mapping[prop - num_static_props][v];
  }

  uint32_t QuantizeStaticProperty(uint32_t prop, pixel_type v) const {
    JXL_DASSERT(prop < num_static_props);
    v = jxl::Clamp1(v, -kPropertyRange, kPropertyRange) + kPropertyRange;
    return static_property_mapping[prop][v];
  }

  // Swaps samples in position a and b. Does nothing if a == b.
  void Swap(size_t a, size_t b);

  void CollectPixelSamples(const Image* images, const ModularOptions *options, size_t start, size_t stop) {
    samples = PixelSamples(images, start, stop,
      options[start].nb_repeats * 0.1f, options[start].max_chan_size,
      FromProperties(props_to_use));
  }

 private:
  TreeSamples() = default;
  // TODO(veluca): as the total number of properties and predictors are known
  // before adding any samples, it might be better to interleave predictors,
  // properties and counts in a single vector to improve locality.
  // A first attempt at doing this actually results in much slower encoding,
  // possibly because of the more complex addressing.
  // Residual information: token and number of extra bits, per predictor.
  std::vector<std::vector<ResidualToken>> residuals;
  // Number of occurrences of each sample.
  std::vector<uint16_t> sample_counts;
  // Quantized static property values
  size_t num_static_props;
  std::array<std::vector<uint32_t>, kNumStaticProperties> static_props;
  // Property values, quantized to at most 256 distinct values.
  std::vector<std::vector<uint8_t>> props;
  // Decompactification info for `props`.
  std::vector<std::vector<int32_t>> compact_properties;
  // List of properties to use.
  std::vector<uint32_t> props_to_use;
  // List of predictors to use.
  std::vector<Predictor> predictors;
  // Mapping property value -> quantized property value.
  static constexpr int32_t kPropertyRange = 511;
  std::array<std::vector<uint16_t>, kNumStaticProperties>
      static_property_mapping;
  std::vector<std::vector<uint8_t>> property_mapping;
  // Number of samples seen.
  size_t num_samples = 0;
  // Table for deduplication.
  static constexpr uint32_t kDedupEntryUnused{static_cast<uint32_t>(-1)};
  std::vector<uint32_t> dedup_table_;

  // Functions for sample deduplication.
  bool IsSameSample(size_t a, size_t b) const;
  size_t Hash1(size_t a) const;
  size_t Hash2(size_t a) const;
  void InitTable(size_t log_size);
  // Returns true if `a` was already present in the table.
  bool AddToTableAndMerge(size_t a);
  void AddToTable(size_t a);

  // Set the predictor to use. Must be called before adding any samples.
  Status SetPredictor(Predictor predictor,
                      ModularOptions::TreeMode wp_tree_mode);
  // Set the properties to use. Must be called before adding any samples.
  Status SetProperties(const std::vector<uint32_t> &properties,
                       ModularOptions::TreeMode wp_tree_mode);

  struct PixelSamples {
    PixelSamples() = default;
    // only random generator depends on `start`
    PixelSamples(const Image* images, size_t start, size_t stop,
                 float fraction, size_t max_chan_size, SampleMask sample_mask) {
      if ((sample_mask & SampleMask::need_sampling) == SampleMask::nothing ||
          stop <= start || fraction <= 0 || fraction > 1)
        return;

      // these vectors are small, so we calculate them always
      group_pixel_count.assign(stop, 0);
      channel_pixel_count.reserve(5);  // ACMetadata channels

      std::vector<const Channel*> channels;

      size_t total_pixels = 0;
      for (size_t group_id = start; group_id < stop; ++group_id) {
        if (images[group_id].channel.size() > channel_pixel_count.size())
          channel_pixel_count.resize(images[group_id].channel.size());

        for (size_t i = 0; i < images[group_id].channel.size(); i++) {
          size_t w = images[group_id].channel[i].w;
          size_t h = images[group_id].channel[i].h;
          if (i >= images[group_id].nb_meta_channels &&
              (w > max_chan_size || h > max_chan_size)) {
            break;
          }
          if (w > 1 && h > 0)  // skip empty or width-1 channels
          {
            size_t pixels = w * h;

            group_pixel_count[group_id] += pixels;
            channel_pixel_count[i] += pixels;

            total_pixels += pixels;
            channels.push_back(&images[group_id].channel[i]);
          }
        }
      }
      total_pixel_count = total_pixels;

      const bool need_pixel = (sample_mask & (SampleMask::pixel | SampleMask::abs_pixel)) != SampleMask::nothing;
      const bool need_diff = (sample_mask & (SampleMask::diff | SampleMask::abs_diff)) != SampleMask::nothing;

      if (total_pixels == 0 || !(need_pixel || need_diff)) return;

      // these vectors are large and produced only when needed, reserve
      // 5 standard deviations larger to account for pseudorandom sampling
      float avg_samples = fraction * total_pixels;
      size_t samples_size =
          avg_samples + 5.0f * sqrtf(avg_samples * (1.0f - fraction));
      if (need_pixel) {
        pixel.reserve(samples_size);
        min_pixel = std::numeric_limits<pixel_type>::max();
        min_abs_pixel = std::numeric_limits<pixel_type>::max();
      }
      if (need_diff) {
        diff.reserve(samples_size);
        min_diff = std::numeric_limits<pixel_type>::max();
        min_abs_diff = std::numeric_limits<pixel_type>::max();
      }

      Rng::GeometricDistribution dist = Rng::MakeGeometric(fraction);
      Rng rng(start);

      for (const Channel* chan : channels) {
        size_t y = 0;
        size_t x = 0;

        auto advance = [&](size_t amount) {
          x += amount;
          // Detect row overflow (rare).
          while (x >= chan->w) {
            x -= chan->w;
            // Detect end-of-channel (even rarer).
            if (++y == chan->h) return true;
          }
          return false;
        };
        // sampling pixels and/or diffs
        for (bool end_reached = advance(fraction <= 0.99 ? rng.Geometric(dist) : 0);
              !end_reached;
              end_reached = advance(1 + (fraction <= 0.99 ? rng.Geometric(dist) : 0))) {
          const pixel_type* row = chan->Row(y);
          if (need_pixel) {
            pixel_type pix = row[x];
            pixel.push_back(pix);
            min_pixel = std::min(min_pixel, pix);
            min_abs_pixel = std::min(min_abs_pixel, std::abs(pix));
          }
          if (need_diff) {
            size_t xp = x == 0 ? 1 : x - 1;
            pixel_type dif = static_cast<pixel_type_w>(row[x]) - row[xp];
            diff.push_back(dif);
            min_diff = std::min(min_diff, dif);
            min_abs_diff = std::min(min_abs_diff, std::abs(dif));
          }
        }
      }
    }

    pixel_type min_pixel;
    pixel_type min_abs_pixel;
    std::vector<pixel_type> pixel;

    pixel_type min_diff;
    pixel_type min_abs_diff;
    std::vector<pixel_type> diff;

    uint64_t total_pixel_count = 0;
    std::vector<uint32_t> group_pixel_count;
    std::vector<uint32_t> channel_pixel_count;
  } samples;
};

Status TokenizeTree(const Tree &tree, std::vector<Token> *tokens,
                    Tree *decoder_tree);

Status ComputeBestTree(TreeSamples &tree_samples, float threshold,
                       const std::vector<ModularMultiplierInfo> &mul_info,
                       StaticPropRange static_prop_range,
                       float fast_decode_multiplier, Tree *tree);

}  // namespace jxl
#endif  // LIB_JXL_MODULAR_ENCODING_ENC_MA_H_
