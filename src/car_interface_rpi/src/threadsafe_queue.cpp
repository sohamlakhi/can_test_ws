#include "car_interface_rpi/threadsafe_queue.hpp"
#include "car_interface_rpi/CarCommandValue.h"
#include "car_interface_rpi/CarCommand.h"

using namespace std;

template <class T> T threadsafe_queue<T>::front() {
    lock_guard<mutex> lg(m);

    T front_value;

    if (!queue.empty()) {
        front_value = queue.front();
    }

    return front_value;
}

template <class T> void threadsafe_queue<T>::push(T element) {
    lock_guard<mutex> lg(m);

    queue.push(element);
    
}

template <class T> void threadsafe_queue<T>::pop() {
    lock_guard<mutex> lg(m);

    if (!queue.empty()) {
        queue.pop();
    }
    
}

template <class T> T threadsafe_queue<T>::dequeue() {
    lock_guard<mutex> lg(m);

    T front_value;

    if (!queue.empty()) {
        front_value = queue.front();
        queue.pop();
    }

    return front_value;
}

template <class T> int threadsafe_queue<T>::size() {
    lock_guard<mutex> lg(m);

    return queue.size();
}

template <class T> bool threadsafe_queue<T>::empty() {
    lock_guard<mutex> lg(m);

    return queue.empty();
}

template <class T> std::queue<T> threadsafe_queue<T>::dequeue_all() {
    lock_guard<mutex> lg(m);

    std::queue<T> popped_queue = queue;
    
    while(!queue.empty()) {
        // popped_queue.push(queue.front());
        queue.pop();    
    }
    return popped_queue;
}

// template <class T> void threadsafe_queue<T>::swap(std::queue<T> swap_queue) {
//     lock_guard<mutex> lg(m);



//     if (!queue.empty()) {
//         queue.pop();
//     }
    
// }

// Instantiate threadsafe_queue for the supported template type parameters
// consider instantiating here with your message object code! add message header and see how to add dependencies in cmake
template class threadsafe_queue<double>;
template class threadsafe_queue<car_interface_rpi::CarCommand>;