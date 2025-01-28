/**
 * @file   range_generator.h
 * @brief  Fake iterator simulating range(start, end)
 * @author Nathan Hughes
 */
#pragma once
#include <iterator>
#include <cstdint>

namespace kimera_pgmo {

class RangeGenerator {
 public:
  RangeGenerator(int64_t size, int64_t start = 0);

  struct Iter {
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = int64_t;
    using pointer = int64_t*;
    using reference = int64_t&;

    Iter(int64_t index);

    const int64_t& operator*() const;
    const int64_t* operator->() const;

    Iter& operator+=(int64_t n);
    Iter operator+(int64_t n) const;
    Iter& operator++();
    Iter operator++(int);

    Iter& operator-=(int64_t n);
    Iter operator-(int64_t n) const;
    Iter& operator--();
    Iter operator--(int);

    std::ptrdiff_t operator-(const Iter& other);
    int64_t operator[](size_t n) const;

    friend bool operator==(const Iter& a, const Iter& b);
    friend bool operator<(const Iter& a, const Iter& b);
    friend bool operator<=(const Iter& a, const Iter& b);
    friend bool operator>(const Iter& a, const Iter& b);
    friend bool operator>=(const Iter& a, const Iter& b);

   private:
    int64_t idx_;
  };

  Iter begin() const;
  Iter end() const;
  Iter cbegin() const;
  Iter cend() const;

  const int64_t size;
  const int64_t start;
};

}  // namespace kimera_pgmo
