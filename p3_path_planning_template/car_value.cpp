#include <iostream>
#include <string>
#include <cmath>

/**
 * A struct for holding information about a car.
 */
struct Car {
    /**
     * The make (brand) of the car.
     */
    std::string make;
    
    /**
     * The model of the car.
     */
    std::string model;

    /**
     * The year the car was made.
     */
    int year;

    /**
     * The current value of the car in dollars.
     */
    float price_dollars;

    /**
     * Whether the car is used.
     */
    bool used = false;
};

/**
 * Calculate the value of the car at some point in the future.
 * @param  car The car to calculate the value of.
 * @param  years_from_now The number of years out to estimate the value.
 * @return  The estimated value.
 * 
 * Calculate the value using value = price_now * e^(-r * t) where r is the depreciation
 * rate and t is the years_from_now. Let's say Mercedes cars have a depreciation of 15%
 * or 0.15, and Teslas have a depreciation of 25% or 0.25. All other cars have a rate of
 * 20% or 0.20.
 */
float calculateCarValue(Car car, float years_from_now) {
    // *** Task: Implement this function *** //
    float depreciation_rate;
    if (car.make == "Mercedes"){
        depreciation_rate = 0.15;
    }
    else if (car.make == "Tesla"){
        depreciation_rate = 0.25;
    }
    else {
        depreciation_rate = 0.20;
    }
    float value = car.price_dollars * pow(exp(1),(-1*depreciation_rate * years_from_now));
    return value; 
    // *** End Student Code *** //
}

int main() {

    // *** Task: Collect information about the car then calculate its value with calculate_car_value() *** //
    std::string make; 
    std::string model;
    int year;
    float price_dollars;
    int years_from_now;

    std::cout << "Car Make: ";
    getline(std::cin, make);
    std::cout << "Car model: ";
    getline(std::cin, model); 
    std::cout << "Year car was made: ";
    std::cin >> year;
    std::cout << "Current price: ";
    std::cin >> price_dollars;
    std::cout << "Years from now: ";
    std::cin >> years_from_now;

    Car car; 
    car.make = make; 
    car.model = model;
    car.year = year;
    car.price_dollars = price_dollars;

    float price = calculateCarValue(car, years_from_now);
    std::cout << price;


    // *** End Student Code *** //

    return 0;
}