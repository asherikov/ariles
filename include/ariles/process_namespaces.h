/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Collects defined config namespaces into a single list.
*/

#pragma once


#ifndef ARILES_NAMESPACE_1
#    define ARILES_NAMESPACE_LIST \
                 ARILES_NAMESPACE(ARILES_NAMESPACE_0)
#else
#    ifndef ARILES_NAMESPACE_2
#        define ARILES_NAMESPACE_LIST \
                     ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                     ARILES_NAMESPACE(ARILES_NAMESPACE_1)
#    else
#        ifndef ARILES_NAMESPACE_3
#            define ARILES_NAMESPACE_LIST \
                         ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                         ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                         ARILES_NAMESPACE(ARILES_NAMESPACE_2)
#        else
#            ifndef ARILES_NAMESPACE_4
#                define ARILES_NAMESPACE_LIST \
                             ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                             ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                             ARILES_NAMESPACE(ARILES_NAMESPACE_2) \
                             ARILES_NAMESPACE(ARILES_NAMESPACE_3)
#            else
#                ifndef ARILES_NAMESPACE_5
#                    define ARILES_NAMESPACE_LIST \
                                 ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                                 ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                                 ARILES_NAMESPACE(ARILES_NAMESPACE_2) \
                                 ARILES_NAMESPACE(ARILES_NAMESPACE_3) \
                                 ARILES_NAMESPACE(ARILES_NAMESPACE_4)
#                else
#                    ifndef ARILES_NAMESPACE_6
#                        define ARILES_NAMESPACE_LIST \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_2) \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_3) \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_4) \
                                     ARILES_NAMESPACE(ARILES_NAMESPACE_5)
#                    else
#                        ifndef ARILES_NAMESPACE_7
#                            define ARILES_NAMESPACE_LIST \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_2) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_3) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_4) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_5) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_6)
#                        else
#                            define ARILES_NAMESPACE_LIST \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_0) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_1) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_2) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_3) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_4) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_5) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_6) \
                                         ARILES_NAMESPACE(ARILES_NAMESPACE_7)
#                        endif
#                    endif
#                endif
#            endif
#        endif
#    endif
#endif
