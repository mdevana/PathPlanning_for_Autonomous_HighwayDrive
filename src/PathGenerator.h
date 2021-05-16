#ifndef PathGenerator_H
#define PathGenerator_H

class PathGenerator {
 public:
  /**
   * Constructor
   */
  PathGenerator();

  /**
   * Destructor.
   */
  virtual ~PathGenerator();
  
  void Init();

  double get_x_vals();
  double get_y_vals();
  
  void generate_simple_path();
  
  

 private:

  vector<double> next_x_vals;
  vector<double> next_y_vals;

};

#endif  // PathGenerator_H