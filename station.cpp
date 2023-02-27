#include "station.h"

Station::Station() {}

Station::Station(Point location, int station_id, int free_spaces) {
    this->station_loc   = location;
    this->station_id    = station_id;
    this->free_spaces   = free_spaces;
    this->is_used       = false;
}

int Station::get_station_id() {return this->station_id;}
void Station::set_station_id(int station_id) {this->station_id = station_id;}

Point Station::get_station_loc() {return this->station_loc;}
void Station::set_station_loc(Point location) {this->station_loc = location;}

int Station::get_free_spaces() {return this->free_spaces;}
void Station::set_free_spaces(int value) {this->free_spaces = value;}

bool Station::get_is_used() {return this->is_used;}
void Station::set_is_used(bool value) {this->is_used = value;}
