/*################################################################################
  ##
  ##   Copyright (C) 2016-2020 Keith O'Hara
  ##
  ##   This file is part of the OptimLib C++ library.
  ##
  ##   Licensed under the Apache License, Version 2.0 (the "License");
  ##   you may not use this file except in compliance with the License.
  ##   You may obtain a copy of the License at
  ##
  ##       http://www.apache.org/licenses/LICENSE-2.0
  ##
  ##   Unless required by applicable law or agreed to in writing, software
  ##   distributed under the License is distributed on an "AS IS" BASIS,
  ##   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  ##   See the License for the specific language governing permissions and
  ##   limitations under the License.
  ##
  ################################################################################*/

/*
 * Differential Evolution (DE) optimization
 */

#ifndef _optim_de_HPP
#define _optim_de_HPP

bool de_int(Vec_t& init_out_vals, 
            std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
            void* opt_data, 
            algo_settings_t* settings_inp);

bool de(Vec_t& init_out_vals, 
        std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
        void* opt_data);

bool de(Vec_t& init_out_vals, 
        std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
        void* opt_data, 
        algo_settings_t& settings);

//

inline
bool
de_int(Vec_t& init_out_vals, 
              std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
              void* opt_data, 
              algo_settings_t* settings_inp)
{
    bool success = false;

    const size_t n_vals = OPTIM_MATOPS_SIZE(init_out_vals);

    //
    // DE settings

    algo_settings_t settings;

    if (settings_inp) {
        settings = *settings_inp;
    }

    const uint_t conv_failure_switch = settings.conv_failure_switch;
    const double err_tol = settings.err_tol;

    const size_t n_pop = settings.de_n_pop;
    const size_t n_gen = settings.de_n_gen;
    const uint_t check_freq = (settings.de_check_freq > 0) ? settings.de_check_freq : n_gen ;

    const uint_t mutation_method = settings.de_mutation_method;

    const double par_F = settings.de_par_F;
    const double par_CR = settings.de_par_CR;

    const Vec_t par_initial_lb = ( OPTIM_MATOPS_SIZE(settings.de_initial_lb) == n_vals ) ? settings.de_initial_lb : OPTIM_MATOPS_ARRAY_ADD_SCALAR(init_out_vals, -0.5);
    const Vec_t par_initial_ub = ( OPTIM_MATOPS_SIZE(settings.de_initial_ub) == n_vals ) ? settings.de_initial_ub : OPTIM_MATOPS_ARRAY_ADD_SCALAR(init_out_vals,  0.5);

    const bool vals_bound = settings.vals_bound;
    
    const Vec_t lower_bounds = settings.lower_bounds;
    const Vec_t upper_bounds = settings.upper_bounds;

    const VecInt_t bounds_type = determine_bounds_type(vals_bound, n_vals, lower_bounds, upper_bounds);

    // lambda function for box constraints

    std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* box_data)> box_objfn \
    = [opt_objfn, vals_bound, bounds_type, lower_bounds, upper_bounds] (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data) \
    -> double
    {
        if (vals_bound) {
            Vec_t vals_inv_trans = inv_transform(vals_inp, bounds_type, lower_bounds, upper_bounds);
            
            return opt_objfn(vals_inv_trans,nullptr,opt_data);
        } else {
            return opt_objfn(vals_inp,nullptr,opt_data);
        }
    };

    //
    // setup

    Vec_t objfn_vals(n_pop);
    Mat_t X(n_pop,n_vals), X_next(n_pop,n_vals);

#ifdef OPTIM_USE_OMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < n_pop; ++i) {
        X_next.row(i) = OPTIM_MATOPS_TRANSPOSE( OPTIM_MATOPS_HADAMARD_PROD( (par_initial_lb + (par_initial_ub - par_initial_lb)), OPTIM_MATOPS_RANDU_VEC(n_vals) ) );

        double prop_objfn_val = opt_objfn( OPTIM_MATOPS_TRANSPOSE(X_next.row(i)), nullptr, opt_data);

        if (!std::isfinite(prop_objfn_val)) {
            prop_objfn_val = inf;
        }
        
        objfn_vals(i) = prop_objfn_val;

        if (vals_bound) {
            X_next.row(i) = OPTIM_MATOPS_TRANSPOSE( transform( OPTIM_MATOPS_TRANSPOSE(X_next.row(i)), bounds_type, lower_bounds, upper_bounds) );
        }
    }

    size_t min_objfn_val_index = index_min(objfn_vals);
    double min_objfn_val = objfn_vals(min_objfn_val_index);

    double min_objfn_val_running = min_objfn_val;
    double min_objfn_val_check   = min_objfn_val;

    RowVec_t best_vec = X_next.row( index_min(objfn_vals) );
    RowVec_t best_sol_running = best_vec;

    //
    // begin loop

    uint_t iter = 0;
    double err = 2*err_tol;

    while (err > err_tol && iter < n_gen + 1) {
        ++iter;

        X = X_next;

        //
        // loop over population

#ifdef OPTIM_USE_OMP
        #pragma omp parallel for
#endif
        for (size_t i = 0; i < n_pop; ++i) {
            uint_t c_1, c_2, c_3;

            do { // 'r_2' in paper's notation
                c_1 = OPTIM_MATOPS_AS_SCALAR( OPTIM_MATOPS_RANDI_VEC(1, 0, n_pop-1) ); // arma::as_scalar(arma::randi(1, arma::distr_param(0, n_pop-1)));
            } while(c_1 == i);

            do { // 'r_3' in paper's notation
                c_2 = OPTIM_MATOPS_AS_SCALAR( OPTIM_MATOPS_RANDI_VEC(1, 0, n_pop-1) );
            } while(c_2==i || c_2==c_1);

            do { // 'r_1' in paper's notation
                c_3 = OPTIM_MATOPS_AS_SCALAR( OPTIM_MATOPS_RANDI_VEC(1, 0, n_pop-1) );
            } while(c_3==i || c_3==c_1 || c_3==c_2);

            //

            const size_t j = OPTIM_MATOPS_AS_SCALAR( OPTIM_MATOPS_RANDI_VEC(1, 0, n_vals-1) ); // arma::as_scalar(arma::randi(1, arma::distr_param(0, n_vals-1)));

            Vec_t rand_unif = OPTIM_MATOPS_RANDU_VEC(n_vals);
            RowVec_t X_prop(n_vals);

            for (size_t k = 0; k < n_vals; ++k) {
                if ( rand_unif(k) < par_CR || k == j ) {
                    if ( mutation_method == 1 ) {
                        X_prop(k) = X(c_3,k) + par_F*(X(c_1,k) - X(c_2,k));
                    } else {
                        X_prop(k) = best_vec(k) + par_F*(X(c_1,k) - X(c_2,k));
                    }
                } else {
                    X_prop(k) = X(i,k);
                }
            }

            //

            double prop_objfn_val = box_objfn( OPTIM_MATOPS_TRANSPOSE(X_prop), nullptr, opt_data);

            if (!std::isfinite(prop_objfn_val)) {
                prop_objfn_val = inf;
            }
            
            if (prop_objfn_val <= objfn_vals(i)) {
                X_next.row(i) = X_prop;
                objfn_vals(i) = prop_objfn_val;
            } else {
                X_next.row(i) = X.row(i);
            }
        }

        min_objfn_val_index = index_min(objfn_vals);
        min_objfn_val = objfn_vals(min_objfn_val_index);

        best_vec = X_next.row( min_objfn_val_index );

        //
        // assign running global minimum

        if (min_objfn_val < min_objfn_val_running) {
            min_objfn_val_running = min_objfn_val;
            best_sol_running = best_vec;
        }

        if (iter%check_freq == 0) {   
            err = std::abs(min_objfn_val_running - min_objfn_val_check) / (1.0 + std::abs(min_objfn_val_running));
            
            if (min_objfn_val_running < min_objfn_val_check) {
                min_objfn_val_check = min_objfn_val_running;
            }
        }
    }

    //

    if (vals_bound) {
        best_sol_running = OPTIM_MATOPS_TRANSPOSE( inv_transform(OPTIM_MATOPS_TRANSPOSE(best_sol_running), bounds_type, lower_bounds, upper_bounds) );
    }

    error_reporting(init_out_vals, OPTIM_MATOPS_TRANSPOSE(best_sol_running), opt_objfn, opt_data, success, err, err_tol, iter, n_gen, conv_failure_switch, settings_inp);

    //
    
    return true;
}

inline
bool
de(Vec_t& init_out_vals, 
          std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
          void* opt_data)
{
    return de_int(init_out_vals, opt_objfn, opt_data, nullptr);
}

inline
bool
de(Vec_t& init_out_vals, 
          std::function<double (const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)> opt_objfn, 
          void* opt_data, 
          algo_settings_t& settings)
{
    return de_int(init_out_vals, opt_objfn, opt_data, &settings);
}

#endif
