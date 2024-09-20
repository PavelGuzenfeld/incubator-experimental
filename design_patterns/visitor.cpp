#include <iostream> // Standard I/O
#include <tuple>

/* VisitorAcceptorBase Template
* Base class for visitable objects that accept visitors.
* Derived classes must implement the accept method.
*/
template <typename DerivedVisitable, typename... Visitors>
class VisitorAcceptorBase
{
public:
    template <typename Visitor>
    constexpr auto accept(Visitor &&visitor) const noexcept
    {
        return std::forward<Visitor>(visitor).visit(static_cast<const DerivedVisitable &>(*this));
    }
};

/* VisitorBase Template
* Base class for visitors that visit visitable objects.
* Derived classes must implement visit methods for each visitable type.
*/
template <typename DerivedVisitor, typename... Visitables>
class VisitorBase
{
public:
    using ReturnType = void; // Default return type is void

    template <typename Acceptor>
    constexpr auto visit(Acceptor &&acceptor) const noexcept -> ReturnType
    {
        return std::forward<Acceptor>(acceptor).accept(static_cast<const DerivedVisitor &>(*this));
    }
};

// --- LiteralExpression (represents numbers)

class LiteralExpression : public VisitorAcceptorBase<LiteralExpression>
{
public:
    int value_;

    LiteralExpression(int value) : value_(value) {}

    int getValue() const
    {
        return value_;
    }
};

// --- AddExpression (represents the addition of two expressions)

class AddExpression : public VisitorAcceptorBase<AddExpression>
{
public:
    const LiteralExpression &lhs_; // Left-hand side expression
    const LiteralExpression &rhs_; // Right-hand side expression

    AddExpression(const LiteralExpression &lhs, const LiteralExpression &rhs)
        : lhs_(lhs), rhs_(rhs) {}

    const LiteralExpression &getLHS() const
    {
        return lhs_;
    }

    const LiteralExpression &getRHS() const
    {
        return rhs_;
    }
};

// CombinedAcceptor Template
template <typename... Visitables>
class CombinedAcceptor : public VisitorAcceptorBase<CombinedAcceptor<Visitables...>>
{
public:
    std::tuple<Visitables &...> visitables_; // Holds references to all visitables

    // Constructor accepts rvalues and lvalues (perfect forwarding)
    CombinedAcceptor(Visitables &...visitables)
        : visitables_(visitables...) {}

    // Accept the visitor and visit each Visitable in the tuple
    template <typename Visitor>
    constexpr auto accept(Visitor &&visitor) const noexcept
    {
        return std::apply([&visitor](auto &...visitables)
                          {
                              (std::forward<Visitor>(visitor).visit(visitables), ...); // C++17 fold expression
                          },
                          visitables_);
    }
};

// --- Visitor without return value (ConcreteVisitor)

class ConcreteVisitor : public VisitorBase<ConcreteVisitor>
{
public:
    // Visit methods without constexpr due to std::cout
    void visit(const class VisitableA &) const noexcept
    {
        std::cout << "ConcreteVisitor is visiting VisitableA.\n";
    }

    void visit(const class VisitableB &) const noexcept
    {
        std::cout << "ConcreteVisitor is visiting VisitableB.\n";
    }

    // Generic visit for anything else, like CombinedAcceptor or new types.
    template <typename CombinedVisitable>
    void visit(const CombinedVisitable &combined) const noexcept
    {
        std::cout << "ConcreteVisitor is visiting a CombinedAcceptor.\n";
        combined.accept(*this); // Recursively visit all
    }
};

// --- Visitor with return value (EvalVisitor)

class EvalVisitor : public VisitorBase<EvalVisitor>
{
public:
    using ReturnType = int; // Override return type for EvalVisitor

    // Visit LiteralExpression to retrieve its value
    int visit(const LiteralExpression &literal) const noexcept
    {
        return literal.getValue(); // Return the literal value
    }

    // Visit AddExpression to evaluate the sum of two literals
    int visit(const AddExpression &addExpr) const noexcept
    {
        int lhsResult = this->visit(addExpr.getLHS()); // Get LHS result
        int rhsResult = this->visit(addExpr.getRHS()); // Get RHS result
        return lhsResult + rhsResult;                  // Return the sum
    }
};

// --- Visitables

class VisitableA : public VisitorAcceptorBase<VisitableA>
{
    // No specific logic here
};

class VisitableB : public VisitorAcceptorBase<VisitableB>
{
    // No specific logic here
};

// --- MathVisitor: Evaluates and Prints the expression (no return value)

class MathVisitor : public VisitorBase<MathVisitor>
{
public:
    // Visit LiteralExpression to retrieve its value
    void visit(const LiteralExpression &literal) const noexcept
    {
        std::cout << literal.getValue(); // Just print the literal value
    }

    // Visit AddExpression to evaluate the sum of two literals
    void visit(const AddExpression &addExpr) const noexcept
    {
        std::cout << "(";
        this->visit(addExpr.getLHS()); // Visit the left-hand side
        std::cout << " + ";
        this->visit(addExpr.getRHS()); // Visit the right-hand side
        std::cout << ")";
    }
};

// --- Main

int main()
{
    // MathVisitor for printing (no return value)
    MathVisitor mathVisitor;

    // Create literal expressions
    LiteralExpression literal1(5);
    LiteralExpression literal2(10);

    // Create an addition expression that adds the two literals
    AddExpression addExpr(literal1, literal2);

    // Visit and print the addition expression
    mathVisitor.visit(addExpr); // Output: (5 + 10)

    std::cout << std::endl;

    // EvalVisitor for evaluating the result (returns the sum)
    EvalVisitor evalVisitor;
    int result = evalVisitor.visit(addExpr); // Should evaluate to 15

    std::cout << "Result of addition: " << result << std::endl; // Output: Result of addition: 15

    ConcreteVisitor visitor; // Create the visitor
    VisitableA visitableA;   // Create VisitableA
    VisitableB visitableB;   // Create VisitableB

    // Create a combined acceptor holding both VisitableA and VisitableB
    CombinedAcceptor combined{visitableA, visitableB};

    // Visit individual visitables (const and non-const lvalues)
    visitor.visit(visitableA); // Output: Visiting VisitableA
    visitor.visit(visitableB); // Output: Visiting VisitableB

    // Visit the combined acceptor, which visits both inside
    visitor.visit(combined);
    // Output:
    // ConcreteVisitor is visiting a CombinedAcceptor
    // ConcreteVisitor is visiting VisitableA
    // ConcreteVisitor is visiting VisitableB

    // Test with nested combined acceptor
    CombinedAcceptor nestedCombined{combined, visitableA};
    visitor.visit(nestedCombined);
    // Output:
    // ConcreteVisitor is visiting a CombinedAcceptor
    // ConcreteVisitor is visiting a CombinedAcceptor
    // ConcreteVisitor is visiting VisitableA
    // ConcreteVisitor is visiting VisitableB
    // ConcreteVisitor is visiting VisitableA

    // Test with const objects
    const VisitableA constVisitableA;
    const VisitableB constVisitableB;
    CombinedAcceptor constCombined{constVisitableA, constVisitableB};
    visitor.visit(constCombined);
    // Output:
    // ConcreteVisitor is visiting a CombinedAcceptor
    // ConcreteVisitor is visiting VisitableA
    // ConcreteVisitor is visiting VisitableB

    return 0;
}
