#ifndef ROVIO_MEMORY_HPP_
#define ROVIO_MEMORY_HPP_

#include <memory>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/StdVector>

template <template <typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;

/// \brief Aligned allocator to be used like std::make_shared
///
/// To be used like:
///   std::shared_ptr my_ptr = aligned_shared<aslam::RadTanDistortion>(params);
template<typename Type, typename ... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                    std::forward<Arguments>(arguments)...);
}

namespace internal {
template <typename Type>
struct aligned_delete {
  constexpr aligned_delete() noexcept = default;

  template <typename TypeUp,
            typename = typename std::enable_if<
                std::is_convertible<TypeUp*, Type*>::value>::type>
  aligned_delete(const aligned_delete<TypeUp>&) noexcept {}

  void operator()(Type* ptr) const {
    static_assert(sizeof(Type) > 0, "Can't delete pointer to incomplete type!");
    typedef typename std::remove_const<Type>::type TypeNonConst;
    Eigen::aligned_allocator<TypeNonConst> allocator;
    allocator.deallocate(ptr, 1u /*num*/);
  }
};
}  // namespace internal

template <typename Type>
using AlignedUniquePtr = std::unique_ptr<
    Type, internal::aligned_delete<typename std::remove_const<Type>::type>>;

template <typename Type, typename... Arguments>
inline AlignedUniquePtr<Type> aligned_unique(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  Eigen::aligned_allocator<TypeNonConst> allocator;
  TypeNonConst* obj = ::new (allocator.allocate(1u))  // NOLINT
      Type(std::forward<Arguments>(arguments)...);
  return std::move(AlignedUniquePtr<Type>(obj));
}

#endif  // ROVIO_MEMORY_HPP_