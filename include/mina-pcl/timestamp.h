#ifndef __CHECKPOINT__
#define __CHECKPOINT__

#include <chrono>
#include <iostream>

class CheckPoint{
    private:
        std::chrono::_V2::system_clock::time_point start;
    public:
        bool checkpoint_enable = true;

        CheckPoint(bool en=true){
            checkpoint_enable = en;
        };
        ~CheckPoint(){};

        void enable(){
            checkpoint_enable = true;
        }

        void disable(){
            checkpoint_enable = false;
        }

        inline void begin(){
            if(checkpoint_enable){
                start = std::chrono::high_resolution_clock::now();
                std::cout << std::endl;
            }
        };

        inline void create(const std::string str){
            if(checkpoint_enable){
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                std::cout << str << (double)duration.count()/1000.0f << "ms" << std::endl;
                start = std::chrono::high_resolution_clock::now();
            }
        }
};
#endif // __CHECKPOINT__