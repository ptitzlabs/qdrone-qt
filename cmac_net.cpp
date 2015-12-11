#include "cmac_net.h"


cmac_net::cmac_net(){}
cmac_net::cmac_net
(int num_inputs, double* tile_dimension, int tile_resolution,
                   int memory_size, int num_tilings, int num_hashings,
                   double alpha, double gamma, double lambda){
    parm_init(num_inputs, tile_dimension, tile_resolution, memory_size,
              num_tilings, num_hashings, alpha, gamma, lambda);
}

void cmac_net::parm_init(int num_inputs, double* tile_dimension,
                         int tile_resolution, int memory_size, int num_tilings,
                         int num_hashings, double alpha, double gamma,
                         double lambda) {
    _cmac_net_parm.memory_size = memory_size;
    _cmac_net_parm.num_tilings = num_tilings;
    _cmac_net_parm.num_hashings = num_hashings;
    _cmac_net_parm.tile_resolution = tile_resolution;
    _cmac_net_parm.alpha = alpha/num_tilings;
    _cmac_net_parm.gamma = gamma;
    _cmac_net_parm.lambda = lambda;
    _cmac_net_parm.num_inputs = num_inputs;
    _cmac_net_parm.weights = new double[memory_size]();
    _cmac_net_parm.tile_dimension = new double[num_inputs];
    _cmac_net_parm.tile_sub_dimension = new double[num_inputs];
    for (int i = 0; i < num_inputs; i++) {
        _cmac_net_parm.tile_dimension[i] = tile_dimension[i];  // tile dimensions are stored
        _cmac_net_parm.tile_sub_dimension[i] =
            tile_dimension[i] /
            (double)tile_resolution;  // tile sub-dimensions are calculated
    }
    map_pointers();
    init_tmp();

}

void cmac_net::init_tmp(){
    _traces_tmp = new double[*_memory_size]();             // initialize traces
    _hashings_tmp = new int* [*_num_hashings];  // initialize hashings
    for (int i = 0; i < *_num_hashings; i++)
        _hashings_tmp[i] =
            new int[*_num_tilings]();  // allocate enough feature hashings to get
                                   // individual tile outputs for each action

}

void cmac_net::map_pointers(){

    // Mapping pointers
    _memory_size = & _cmac_net_parm.memory_size;
    _num_tilings = & _cmac_net_parm.num_tilings;
    _num_hashings = & _cmac_net_parm.num_hashings;
    _num_inputs = &_cmac_net_parm.num_inputs;
    _alpha = &_cmac_net_parm.alpha;
    _gamma = &_cmac_net_parm.gamma;
    _lambda = &_cmac_net_parm.lambda;
    _tile_dimension = new double*[*_num_inputs];
    for (int i = 0; i < *_num_inputs; i++)
        _tile_dimension[i] = & _cmac_net_parm.tile_dimension[i];
    _tile_sub_dimension = new double*[*_num_inputs];
    for (int i = 0; i < *_num_inputs; i++)
        _tile_sub_dimension[i] = & _cmac_net_parm.tile_sub_dimension[i];

    _tile_resolution = & _cmac_net_parm.tile_resolution;
    _weights = new double*[*_memory_size];
    for (int i = 0; i < *_memory_size; i++)
        _weights[i] = & _cmac_net_parm.weights[i];
    _max_num_vars = &_cmac_net_parm.max_num_vars;
}

cmac_net::~cmac_net() {
//    delete[] _traces_tmp;
//    _traces_tmp = NULL;
//    for (int i = 0; i < *_num_hashings; i++) {
//        delete[] _hashings_tmp[i];
//        _hashings_tmp[i] = NULL;
//    }
//    delete[] _hashings_tmp;
//    _hashings_tmp = NULL;
}

cmac_net& cmac_net::operator=(const cmac_net& source){
//    source.clone_parm(&this->_cmac_net_parm);
    _cmac_net_parm = source._cmac_net_parm;
    map_pointers();
    init_tmp();
    return *this;
}

void cmac_net::clone_cmac_net_parm(cmac_net_parm * target) const{
    *target = _cmac_net_parm;
}

void cmac_net::clear_traces() {
    for (int i = 0; i < *_memory_size; i++) _traces_tmp[i] = 0.0f;
}
void cmac_net::clear_weights() {
    for (int i = 0; i < *_memory_size; i++) *_weights[i] = 0.0f;
}
void cmac_net::drop_traces() {
    for (int i = 0; i < *_memory_size; i++) _traces_tmp[i] *= *_gamma * *_lambda;
}

void cmac_net::update_traces(int hash) {
    for (int i = 0; i < *_num_hashings; i++) {
        if (i != hash) {
            for (int j = 0; j < *_num_tilings; j++) {
                _traces_tmp[_hashings_tmp[i][j]] = 0.0f;
            }
        }
    }
    for (int i = 0; i < *_num_tilings; i++) _traces_tmp[_hashings_tmp[hash][i]] = 1.0f;
}
void cmac_net::generate_tiles(double* input) {
    double input_tmp[*_num_inputs];
    for (int i = 0; i < *_num_inputs; i++) {
        input_tmp[i] = input[i] / *_tile_sub_dimension[i];
    }
    for (int i = 0; i < *_num_hashings; i++) {
        get_tiles(_hashings_tmp[i], *_num_tilings, input_tmp, *_num_inputs,
                   *_memory_size, i);
    }
}

void cmac_net::return_value(double* output, int hash) {
    *output = 0;
    for (int i = 0; i < *_num_tilings; i++)
        *output += *_weights[_hashings_tmp[hash][i]];
}
void cmac_net::quick_update(double delta) {
    double tmp = *_alpha * delta;
    for (int i = 0; i < *_memory_size; i++) {
        *_weights[i] += tmp * _traces_tmp[i];
    }
}
void cmac_net::update(double* input, double target, int hash) {
    int tiles_array[*_num_tilings];
    get_tiles(tiles_array, *_num_tilings, input, *_num_inputs, *_memory_size,
               hash);
    double output = 0;

    for (int i = 0; i < *_num_tilings; i++) output += *_weights[tiles_array[i]];

    double err = target - output;
    double delta = err * *_alpha;
    for (int i = 0; i < *_num_tilings; i++) {
        *_weights[tiles_array[i]] += delta;
    }
}

void cmac_net::report() {
    printf("\n##############\n");
    printf("CMAC report:\n");
    printf("Number of inputs:  %i\n", *_num_inputs);
    printf("Tile dimension: ");
    for (int i = 0; i < *_num_inputs; i++)
        printf("%i: (%f) ", i, *_tile_dimension[i]);
    printf("\n");
    printf("Number of tilings:  %i\n", *_num_tilings);
    printf("Memory size:       %i\n", *_memory_size);
    printf("alpha:              %.2f\n", *_alpha * *_num_tilings);
}

int cmac_net::get_num_hashings() { return *_num_hashings; }
double* cmac_net::get_weights() { return *_weights; }
double cmac_net::get_weight(int n) { return 100; }
int cmac_net::get_memory_size() { return *_memory_size; }
int cmac_net::get_num_tilings() { return *_num_tilings; }

double* cmac_net::get_tile_sub_dimension() { return *_tile_sub_dimension; }

void cmac_net::write_weights(char* filename) {
    // int file = open(filename, O_BINARY | O_WRONLY);
    // write(file,(char *)_weights, _memory_size*sizeof(double));
    // close(file);
    FILE* weights_file = fopen(filename, "wb");
    fwrite(*_weights, sizeof(double), *_memory_size, weights_file);
    fclose(weights_file);
}
void cmac_net::read_weights(char* filename) {
    double weights[*_memory_size];
    FILE* weights_file = fopen(filename, "rb");
    fread(weights, sizeof(double), *_memory_size, weights_file);
    fclose(weights_file);

    for (int i = 0; i < *_memory_size; i++) {
        std::cout << weights[i] << std::endl;
    }
    // int file = open(filename, O_BINARY | O_RDONLY);
    // read(file,(char *)_weights, _memory_size*sizeof(double));
    // close(file);
}

void cmac_net::get_tiles(
        int tiles[],        // provided array contains returned tiles (tile indices)
        int num_tilings,    // number of tile indices to be returned in tiles
        double variables[],  // array of variables
        int num_variables,  // number of variables
        int memory_size,    // total number of possible tiles (memory size)
        int hash1,          // change these from -1 to get a different hashing
        int hash2, int hash3) {
        int qstate[*_max_num_vars];
        int base[*_max_num_vars];
        int coordinates[*_max_num_vars +
                        4]; /* one interval number per rel dimension */
        int num_coordinates;
        int i,j;

        if (hash1 == -1)
            num_coordinates =
                num_variables + 1;  // no additional hashing corrdinates
        else if (hash2 == -1) {
            num_coordinates =
                num_variables + 2;  // one additional hashing coordinates
            coordinates[num_variables + 1] = hash1;
        } else if (hash3 == -1) {
            num_coordinates =
                num_variables + 3;  // two additional hashing coordinates
            coordinates[num_variables + 1] = hash1;
            coordinates[num_variables + 2] = hash2;
        } else {
            num_coordinates =
                num_variables + 4;  // three additional hashing coordinates
            coordinates[num_variables + 1] = hash1;
            coordinates[num_variables + 2] = hash2;
            coordinates[num_variables + 3] = hash3;
        }

        /* quantize state to integers (henceforth, tile widths == num_tilings) */
        for (i = 0; i < num_variables; i++) {
            qstate[i] = (int)floor(variables[i] * num_tilings);
            base[i] = 0;
        }

        /*compute the tile numbers */
        for (j = 0; j < num_tilings; j++) {
            /* loop over each relevant dimension */
            for (int i = 0; i < num_variables; i++) {
                /* find coordinates of activated tile in tiling space */
                if (qstate[i] >= base[i])
                    coordinates[i] =
                        qstate[i] - ((qstate[i] - base[i]) % num_tilings);
                else
                    coordinates[i] = qstate[i] + 1 +
                                     ((base[i] - qstate[i] - 1) % num_tilings) -
                                     num_tilings;

                /* compute displacement of next tiling in quantized space */
                base[i] += 1 + (2 * i);
            }
            /* add additional indices for tiling and hashing_set so they hash
             * differently */
            coordinates[i++] = j;

            tiles[j] = hash_coordinates(coordinates, num_coordinates, memory_size);
        }
        return;
    }

/* hash_coordinates
   Takes an array of integer coordinates and returns the corresponding tile
   after hashing
*/
int cmac_net::hash_coordinates(int *coordinates, int num_indices, int memory_size) {
    static int first_call = 1;
    static unsigned int rndseq[2048];
    /*int i,k;*/
    long index;
    long sum = 0;

    /* if first call to hashing, initialize table of random numbers */
    if (first_call) {
        for (int k = 0; k < 2048; k++) {
            rndseq[k] = 0;
            for (unsigned int i = 0; i < sizeof(int); ++i)
                rndseq[k] = (rndseq[k] << 8) | (rand() & 0xff);
        }
        first_call = 0;
    }

    for (int i = 0; i < num_indices; i++) {
        /* add random table offset for this dimension and wrap around */
        index = coordinates[i];
        index += (449 * i);
        index %= 2048;
        while (index < 0) index += 2048;

        /* add selected random number to sum */
        sum += (long)rndseq[(int)index];
    }
    index = (int)(sum % memory_size);
    while (index < 0) index += memory_size;

    return (index);
}

void cmac_net::clone_parm(cmac_net_parm * net_parm)const {
    *net_parm = _cmac_net_parm;
}

void cmac_net::clone_weights(cmac_net * net){
    net->set_weights(*_weights);
}

void cmac_net::set_weights(double * weights){
    for(int i = 0; i < *_memory_size; i++)
        *_weights[i] = weights[i];
}

cmac_net_parm cmac_net::get_cmac_net_parm(){
    return _cmac_net_parm;
}
