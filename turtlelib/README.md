# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   1. Calculate the magnitude of the vector and divide the x and y component by the magnitude. Pass in a vector directly.
   2. Normalize the components and pass those into the normalize function.
   3. Reference the vector to be normalized.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   1. Maintain the full value of the vector but this may take too much space.
   2. Potential issue with making sure the values are updated if they are changed.
   3. Lose some of the value of the vector but you save space.

   - Which of the methods would you implement and why?
   
   I am implementing the first method because it's the most straightforward method.

2. What is the difference between a class and a struct in C++?

- Classes have private members while structs have public members.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

- Transform2D's are a class because they have elements that you would not want a user to be able to modify. However, a Vector2D is simple and needs to be modified and accessed by the user frequently. The C++ guidelines mention that classes should be used if the class has an invariant whereas a struct can be used if members can be used independently of eachother.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

- C++ guidelines explain that single argument constructors should be declared explicitly which is why Vector2D is a struct.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

- inv() is const because it does not change the Transform2D, but instead returns a new Transform2D object. 

- The *=operator is not const because it changed the Transform2D as it is multiplying it by another Transform2D.

- C++ guidelines suggest to make member functions constant.