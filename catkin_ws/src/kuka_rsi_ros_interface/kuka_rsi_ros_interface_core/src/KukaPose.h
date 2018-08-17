#ifndef KUKAPOSE_H
#define KUKAPOSE_H

/*
 * TODO: Consider if this could be replaced by a struct - although private
 * data members are intended it would be simpler.
 *
 * A robot position with angles
 *     {A1: 0, A2: -90, A3: 90, A4: 0, A5: 90, A6: 0}
 * would have the following values:
 *     x: 320.0
 *     y: 0.0
 *     z: 550.0
 *     a: 0.0?
 *     b: 0.0?
 *     c: 0.0?
 */
class KukaPose
{
    private:
        /*
         * The cartesian X co-ordinate of the position.
         */
        double x;

        /*
         * The cartesian Y co-ordinate of the position.
         */
        double y;

        /*
         * The cartesian Z co-ordinate of the position.
         */
        double z;

        /*
         * TODO: check terminology in KUKA docs.
         * The A orientation value of the position.
         */
        double a;

        /*
         * TODO: check terminology in KUKA docs.
         * The B orientation value of the position.
         */
        double b;

        /*
         * TODO: check terminology in KUKA docs.
         * The C orientation value of the position.
         */
        double c;

    public:
        /*
         * Constructor.
         */
        KukaPose(double x, double y, double z, double a, double b, double c);
        
        /*
         * Destructor.
         */
        ~KukaPose();

        /*
         * Returns the X value of the position.
         */
        double getX();

        /*
         * Returns the Y value of the position.
         */
        double getY();

        /*
         * Returns the Z value of the position.
         */
        double getZ();

        /*
         * Returns the A value of the position.
         */
        double getA();

        /*
         * Returns the B value of the position.
         */
        double getB();

        /*
         * Returns the C value of the position.
         */
        double getC();

        /*
         * Returns true if the values of X, Y, Z, A, B, C are all 0, and
         * false otherwise.
         */
        bool isZero();
};

#endif /* KukaPose */