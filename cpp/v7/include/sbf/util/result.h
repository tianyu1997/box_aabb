#pragma once
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace sbf::util {

// Lightweight Result<T, E> alternative to exceptions for hot paths.
template <typename T, typename E = std::string>
class Result {
public:
    static Result ok(T val) { return Result(std::move(val), Tag::Ok); }
    static Result err(E msg) { return Result(std::move(msg), Tag::Err); }

    bool is_ok() const { return data_.index() == 0; }

    T& value() {
        if (!is_ok()) throw std::logic_error("Result::value() on err");
        return std::get<0>(data_);
    }
    const T& value() const {
        if (!is_ok()) throw std::logic_error("Result::value() on err");
        return std::get<0>(data_);
    }
    const E& error() const {
        if (is_ok()) throw std::logic_error("Result::error() on ok");
        return std::get<1>(data_);
    }

private:
    enum class Tag { Ok, Err };
    Result(T val, Tag) : data_(std::in_place_index<0>, std::move(val)) {}
    Result(E msg, Tag) : data_(std::in_place_index<1>, std::move(msg)) {}

    std::variant<T, E> data_;
};

}  // namespace sbf::util
