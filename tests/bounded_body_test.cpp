#include "bounded_body.h"
#include <cassert>
#include <cstring>
#include <algorithm>
int main(){
    const char* form="action=arm&source=gamepad";
    char body[64]={};std::size_t offset=0;
    assert(ReadBoundedBody(body,sizeof(body),std::strlen(form),[&](char* dest,std::size_t remaining){
        const auto count=std::min<std::size_t>(remaining,3);
        std::memcpy(dest,form+offset,count);offset+=count;return static_cast<int>(count);
    }));
    assert(std::strcmp(body,form)==0);
    int attempt=0;
    assert(!ReadBoundedBody(body,sizeof(body),std::strlen(form),[&](char* dest,std::size_t){
        if(++attempt==1){std::memcpy(dest,form,10);return 10;}return -1;
    }));
    assert(!ReadBoundedBody(body,sizeof(body),64,[](char*,std::size_t){assert(false);return 0;}));
}
