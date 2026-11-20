#include "app.hpp"
#include <exception>
#include <iostream>

int
main(void)
{
    try {
        Game game;
        game.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}
