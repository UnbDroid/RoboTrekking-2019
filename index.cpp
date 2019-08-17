#include "index.h"

using namespace std;

index::index(uint32_t _limit, uint32_t start){
    limit = _limit;
    pos = start;
}

uint32_t index::idx(int32_t displacement){
    if(displacement<0){
        displacement = -(-displacement%limit);
        return ( ((int32_t)pos+displacement < 0) ? limit + (pos+displacement) : pos+displacement );
    }
    else
        return ( (pos+displacement >= limit) ? (pos+displacement)%limit : pos+displacement );
}

void index::set_pos(uint32_t _pos){
    pos = _pos;
}

index index::operator++(int32_t){
    pos = (pos+1)%limit;
    return *this;
}