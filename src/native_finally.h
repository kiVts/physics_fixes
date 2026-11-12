#pragma once
// Keep this frame free of C++ objects requiring unwinding. Under /EHsc a C++
// destructor alone does not guarantee cleanup for Windows structured exceptions.
// Do not swallow the exception: the caller retains its original failure policy.
template<class Body, class Cleanup>
auto native_finally(Body&& body, Cleanup&& cleanup) -> decltype(body())
{
    __try { return body(); }
    __finally { cleanup(); }
}
