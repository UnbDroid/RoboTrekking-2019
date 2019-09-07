#ifndef INDEX_H_
#define INDEX_H_

#include <stdint.h>

using namespace std;

class index{
    public:
        index(uint32_t _limit, uint32_t start=0);
        uint32_t idx(int32_t displacement=0);
        void set_pos(uint32_t _pos);

        index operator++(int32_t);

    private:
        uint32_t pos;
        uint32_t limit;
};

#endif