<!--
#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Thomas Lobig
#********************************************************************************
-->

# How to document source code

- use visual studio code doxygen plugin: [Doxygen Documentation Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)
- use javadoc doxygen style
- priority 0: document every class
- priority 1: document public functions, brief
- priority 2: document function parameters
- priority 3: document member variables

## Example of Doxygen Javadoc Example, copied from [doxygen.nl](http://www.doxygen.nl/manual/docblocks.html)

~~~c++
/**
 *  A test class. A more elaborate class description.
 */

class Javadoc_Test
{
  public:

    /**
     * An enum.
     * More detailed enum description.
     */

    enum TEnum {
          TVal1, /**< enum value TVal1. */  
          TVal2, /**< enum value TVal2. */  
          TVal3  /**< enum value TVal3. */  
         }
       *enumPtr, /**< enum pointer. Details. */
       enumVar;  /**< enum variable. Details. */

      /**
       * A constructor.
       * A more elaborate description of the constructor.
       */
      Javadoc_Test();

      /**
       * A destructor.
       * A more elaborate description of the destructor.
       */
     ~Javadoc_Test();

      /**
       * a normal member taking two arguments and returning an integer value.
       * @param a an integer argument.
       * @param s a constant character pointer.
       * @see Javadoc_Test()
       * @see ~Javadoc_Test()
       * @see testMeToo()
       * @see publicVar()
       * @return The test results
       */
       int testMe(int a,const char *s);

      /**
       * A pure virtual member.
       * @see testMe()
       * @param c1 the first argument.
       * @param c2 the second argument.
       */
       virtual void testMeToo(char c1,char c2) = 0;

      /**
       * a public variable.
       * Details.
       */
       int publicVar;

      /**
       * a function variable.
       * Details.
       */
       int (*handler)(int a,int b);
};
~~~