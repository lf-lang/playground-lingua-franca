#pragma once

#include <atomic>

template<class T>
class Response {
    private:
    unsigned long uid_;
    T data_;    

    public:
    Response(unsigned long uid, const T& data) 
        : uid_(uid), data_(data) {}

    auto uid() const -> unsigned long { return uid_; }
    auto data() -> T& { return data_; }
    auto data() const -> const T& { return data_; }
};

template<class T>
class Request {
    private:
    static std::atomic<unsigned long> uid_counter;
    
    unsigned long uid_;
    T data_;

    public:
    Request(const T& data) 
        : uid_(uid_counter.fetch_add(1, std::memory_order_relaxed)),
        data_(data) {}

    // shouldn't be used, but is needed to store it as context in a map
    Request() : uid_(0), data_() {} 

    auto uid() const -> unsigned long { return uid_; }
    auto data() -> T& { return data_; }
    auto data() const -> const T& { return data_; }

    template<class U>
    auto make_response() const -> Response<U> { return Response(uid_, U{}); }

    template<class U>
    auto make_response(const U& data) const -> Response<U> { return Response(uid_, U{data}); }
};

template<class T>
std::atomic<unsigned long> Request<T>::uid_counter{1};