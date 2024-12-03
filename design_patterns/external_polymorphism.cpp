#include <concepts>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>

class [[nodiscard]] ShapeConcept
{
public:
    virtual ~ShapeConcept() noexcept = default;

    virtual void serialize(std::ostream &out) const = 0;
    virtual void draw() const = 0;

    // prevent slicing, because apparently, that's what you kids do.
    ShapeConcept() = default;
    ShapeConcept(const ShapeConcept &) = delete;
    ShapeConcept &operator=(const ShapeConcept &) = delete;
    ShapeConcept(ShapeConcept &&) noexcept = delete;
    ShapeConcept &operator=(ShapeConcept &&) noexcept = delete;
};

template <typename GeomShape, typename DrawStrategy>
class [[nodiscard]] ShapeModel final : public ShapeConcept
{
public:
    explicit ShapeModel(GeomShape shape, DrawStrategy strategy)
        : shape_(std::move(shape)), strategy_(std::move(strategy)) {}

    void serialize(std::ostream &out) const override
    {
        serialize_impl(out, shape_);
    }

    void draw() const override
    {
        strategy_(shape_);
    }

private:
    [[nodiscard]] const GeomShape &shape() const noexcept { return shape_; }

    GeomShape shape_;
    DrawStrategy strategy_;

    // in case your fancy polymorphic setup needs different serialize overloads
    template <typename T>
    static void serialize_impl(std::ostream &out, const T &shape)
    {
        out << shape;
    }
};

// type-erasure container because who cares about specifics?
class [[nodiscard]] Shape
{
public:
    template <typename GeomShape, typename DrawStrategy>
    Shape(GeomShape shape, DrawStrategy strategy)
        : concept_(std::make_unique<ShapeModel<GeomShape, DrawStrategy>>(std::move(shape), std::move(strategy))) {}

    void serialize(std::ostream &out) const
    {
        concept_->serialize(out);
    }

    void draw() const
    {
        concept_->draw();
    }

private:
    std::unique_ptr<ShapeConcept> concept_;
};

// mocking some drawing strategies, because clearly, you can't handle it.
struct ConsoleDraw
{
    template <typename T>
    void operator()(const T &shape) const
    {
        std::cout << "Drawing shape: " << shape << '\n';
    }
};

// yes, even the simplest struct gets roasted here
struct Circle
{
    double radius;

    friend std::ostream &operator<<(std::ostream &os, const Circle &c)
    {
        return os << "Circle with radius: " << c.radius;
    }
};

int main()
{
    Circle circle{5.0};
    Shape s(circle, ConsoleDraw{});

    s.serialize(std::cout);
    s.draw();

    return 0;
}
