#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>

enum Axis
{
    X,
    Y,
    Z
};

struct Product
{
    std::array<uint8_t, 3> axis;
    float value = 1.0f;
};

int main()
{
    uint32_t degree = 3;
    std::array<glm::vec3, 8> pos = {
        glm::vec3(0.0f, 0.0, 0.0f),
        glm::vec3(1.0f, 0.0, 0.0f),
        glm::vec3(0.0f, 1.0, 0.0f),
        glm::vec3(1.0f, 1.0, 0.0f),

        glm::vec3(0.0f, 0.0, 1.0f),
        glm::vec3(1.0f, 0.0, 1.0f),
        glm::vec3(0.0f, 1.0, 1.0f),
        glm::vec3(1.0f, 1.0, 1.0f)
    };
    std::array<std::array<uint8_t, 3>, 8> equations = 
    {
		std::array<uint8_t, 3>{ {0, 0, 0} },
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},

        {1, 1, 0},
        {1, 0, 1},
        {0, 1, 1},
        {1, 1, 1}
    };

    // std::array<std::array<uint8_t, 3>, 8> equations = 
    // {
	// 	std::array<uint8_t, 3>{ {0, 0, 0} },
    //     {1, 0, 0},
    //     {0, 1, 0},
    //     {0, 0, 1},
    // };
    std::vector<Product> polynomialElements;

    for(uint32_t k=0; k <= degree; k++)
    {
        for(uint32_t j=0; j <= degree; j++)
        {
            for(uint32_t i=0; i <= degree; i++)
            {
                Product p;
                p.axis[0] = i;
                p.axis[1] = j;
                p.axis[2] = k;
                p.value = 1.0f;

                polynomialElements.push_back(p);
            }
        }
    }

    std::vector<Product> polynomialCache;
    Eigen::MatrixXf A(equations.size() * pos.size(), polynomialElements.size());

    uint32_t rowIndex = 0;
    for(const std::array<uint8_t, 3>& eq : equations)
    {
        for(const glm::vec3& p : pos)
        {
            polynomialCache = polynomialElements;
            for(uint32_t ax=0; ax < 3; ax++)
            {
                for(uint32_t n=0; n < eq[ax]; n++)
                {
                    for(Product& prod : polynomialCache)
                    {
                        if(prod.axis[ax] > 0)
                        {
                            prod.value *= prod.axis[ax];
                            prod.axis[ax]--;
                        } else prod.value = 0.0f;
                    }
                }
            }

            for(uint32_t col=0; col < polynomialCache.size(); col++)
            {
                float res = 1.0f;
                for(uint32_t ax=0; ax < 3; ax++)
                {
					if (polynomialCache[col].axis[ax] > 1e-5f)
					{
						res *= glm::pow(p[ax], polynomialCache[col].axis[ax]);
					}
                }
                res *= polynomialCache[col].value;
                A(rowIndex, col) = res;
            }

            rowIndex++;
        }
    }

    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A ,Eigen::ComputeThinU  | Eigen::ComputeThinV);
    // Eigen::MatrixXf invA = svd.matrixV() *  (svd.singularValues().array().abs() > 0.001f).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    Eigen::MatrixXf invA = A.inverse();
    std::cout << "Matrix A\n" << invA << std::endl;

    std::cout << std::endl;

    std::cout << "inline void calculateCoeff(const std::array<float," << (equations.size() * pos.size())
              << ">& inValues, std::array<float," << polynomialElements.size() << ">& outCoeff) {" << std::endl;
    
    for(uint32_t row=0; row < invA.rows(); row++)
    {
        std::cout << "\toutCoeff[" << row << "] = ";
        for(uint32_t col=0; col < invA.cols(); col++)
        {
            if(invA(row, col) > 0.001f)
            {
                std::cout << invA(row, col) << " * inValues[" << col << "] + ";
            }
        }

        std::cout << "0.0f;" << std::endl;
    }

    std::cout << "}" << std::endl << std::endl << std::endl;

    std::cout << "inline float interpolateValue(const std::array<float," << polynomialElements.size()
              << ">& values, glm::vec3 fracPart) {" << std::endl;

    std::cout << "\treturn 0.0f" << std::endl;
    uint32_t index=0;
    for(uint32_t k=0; k <= degree; k++)
    {
        std::cout << "\t";
        for(uint32_t j=0; j <= degree; j++)
        {
            for(uint32_t i=0; i <= degree; i++)
            {
                std::cout << " + values[" << (index++) << "]";
                for(uint32_t n=0; n < i; n++) std::cout << " * fracPart[0]";
                for(uint32_t n=0; n < j; n++) std::cout << " * fracPart[1]";
                for(uint32_t n=0; n < k; n++) std::cout << " * fracPart[2]";
            }
        }
        std::cout << std::endl;
    }

    std::cout << ";" << std::endl;
    std::cout << "}" << std::endl;
}