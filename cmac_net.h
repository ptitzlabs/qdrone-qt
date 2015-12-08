#ifndef _CMAC_NET_H_
#define _CMAC_NET_H_
#include "RL_headers.h"

struct cmac_net_parm{
    int memory_size;
    int num_tilings;
    int num_hashings;
    int num_inputs;
    double alpha;
    double gamma;
    double lambda;
    double * tile_dimension;
    double * tile_sub_dimension;
    double * weights;
    int tile_resolution;
    int max_num_vars;

    cmac_net_parm();
    ~cmac_net_parm();

    cmac_net_parm& operator=(const cmac_net_parm& source);
};


class cmac_net {
   public:
       cmac_net();
    cmac_net(int num_inputs, double* tile_dimension, int tile_resolution = 8,
             int memory_size = 3000, int num_tilings = 10, int num_hashings = 3,
             double alpha = 0.5, double gamma = 1, double lambda = 0.9);

    cmac_net& operator=(const cmac_net& source);

    
    ~cmac_net();

//    cmac_net& operator=(const cmac_net& net);

    void clone_parm(cmac_net_parm * net_parm) const;
    void clone_weights(cmac_net * net);
    void set_weights(double * weights);

    void parm_init(int num_inputs, double* tile_dimension, int tile_resolution,
             int memory_size, int num_tilings, int num_hashings,
             double alpha , double gamma , double lambda );

    void clear_traces();
    void clear_weights();
    void drop_traces();
    void update_traces(int hash);
    void generate_tiles(double* input);
    void return_value(double* output, int hash);
    void quick_update(double delta);
    void update(double* state, double target, int hash);
    void report();
    int get_num_hashings();
    double * get_weights();
    double * get_tile_sub_dimension();
    int get_memory_size();
    int get_num_tilings();

    void read_weights(char* filename);
    void write_weights(char* filename);
    int id;
    void clone_cmac_net_parm(cmac_net_parm * target)const;

    // Tiles stuff
    void get_tiles(int tiles[],int num_tilings,double variables[], int num_variables, int memory_size, int hash1 = -1, int hash2 = -1, int hash3 = -1);
    int hash_coordinates(int *coordinates, int num_indices, int memory_size);

   cmac_net_parm& get_cmac_net_parm();

   private:

    void map_pointers();
    void init_tmp();

    cmac_net_parm _cmac_net_parm;

    int * _memory_size;
    int *_num_tilings;
    int *_num_hashings;
    int * _num_inputs;
    double * _alpha;
    double * _gamma;
    double * _lambda;
    double** _tile_dimension;
    double** _tile_sub_dimension;
    double** _weights;
    int* _tile_resolution;
    int* _max_num_vars;

    double* _outputs_tmp;
    int** _hashings_tmp;
    double* _traces_tmp;


};
#endif
