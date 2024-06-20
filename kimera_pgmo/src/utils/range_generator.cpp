/**
 * @file   range_generator.cpp
 * @brief  Fake iterator simulating range(start, end)
 * @author Nathan Hughes
 */
#include "kimera_pgmo/utils/range_generator.h"

namespace kimera_pgmo {

using Iter = RangeGenerator::Iter;

RangeGenerator::RangeGenerator(int64_t size, int64_t start)
    : size(size), start(start) {}

Iter RangeGenerator::begin() const { return Iter(start); }

Iter RangeGenerator::end() const { return Iter(size); }

Iter RangeGenerator::cbegin() const { return Iter(start); }

Iter RangeGenerator::cend() const { return Iter(size); }

RangeGenerator::Iter::Iter(int64_t index) : idx_(index) {}

const int64_t& RangeGenerator::Iter::operator*() const { return idx_; }

const int64_t* RangeGenerator::Iter::operator->() const { return &idx_; }

Iter& RangeGenerator::Iter::operator+=(int64_t n) {
  idx_ += n;
  return *this;
}

Iter RangeGenerator::Iter::operator+(int64_t n) const { return Iter(idx_ + n); }

Iter& RangeGenerator::Iter::operator++() {
  idx_++;
  return *this;
}

Iter RangeGenerator::Iter::operator++(int) {
  Iter tmp = *this;
  ++(*this);
  return tmp;
}

Iter& RangeGenerator::Iter::operator-=(int64_t n) {
  idx_ -= n;
  return *this;
}

Iter RangeGenerator::Iter::operator-(int64_t n) const { return Iter(idx_ - n); }

Iter& RangeGenerator::Iter::operator--() {
  idx_--;
  return *this;
}

Iter RangeGenerator::Iter::operator--(int) {
  Iter tmp = *this;
  --(*this);
  return tmp;
}

std::ptrdiff_t RangeGenerator::Iter::operator-(const Iter& other) {
  return idx_ - other.idx_;
}

int64_t RangeGenerator::Iter::operator[](size_t n) const { return n; }

bool operator==(const Iter& a, const Iter& b) { return a.idx_ == b.idx_; };

bool operator<(const Iter& a, const Iter& b) { return a.idx_ < b.idx_; };

bool operator<=(const Iter& a, const Iter& b) { return a.idx_ <= b.idx_; };

bool operator>(const Iter& a, const Iter& b) { return a.idx_ > b.idx_; };

bool operator>=(const Iter& a, const Iter& b) { return a.idx_ >= b.idx_; };

}  // namespace kimera_pgmo
