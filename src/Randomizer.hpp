#ifndef RANDOMIZER_HPP
#define RANDOMIZER_HPP

class Randomizer {
public:
    Randomizer(int seed);
    ~Randomizer();

    void RegisterRandomization();
    void Step();

private:
    int seed;
};

#endif // RANDOMIZER_HPP
