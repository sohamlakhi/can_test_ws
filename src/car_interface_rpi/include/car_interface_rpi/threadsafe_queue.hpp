#pragma once

#include <queue>
#include <mutex>

template <class T> class threadsafe_queue {
    public:
        //constructor
        threadsafe_queue() = default;

        T front();

        int size();

        bool empty();

        void push(T element);

        void pop();

        std::queue<T> dequeue_all();

        /*
            @brief -> retrieve front value and pop element
        */
        T dequeue();

        //void swap(std::queue<T> swap_queue);

        // ~threadsafe_queue();

    private:

        std::queue<T> queue;
        std::mutex m;

};

//TODO: you can use object/struct return type to return an object with boolean and front_value for tryDequeue