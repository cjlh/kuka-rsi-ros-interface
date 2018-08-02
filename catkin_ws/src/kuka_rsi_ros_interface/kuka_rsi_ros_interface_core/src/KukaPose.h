#ifndef KUKAPOSE_H
#define KUKAPOSE_H

/*
 * TODO: Consider if this could be replaced by a struct - although private
 * data members are intended it would be simpler.
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
};

#endif /* KukaPose */