/*
 * Copyright (c) 2013 Miguel Sarabia del Castillo
 * Imperial College London
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*
 * Code based on code from the Boost Libraries:
 * Copyright Kevlin Henney, 2000, 2001, 2002. All rights reserved.
 *
 * Distributed under the Boost Software License, Version 1.0. (See
 * accompanying file LICENSE_1_0.txt or copy at
 * http://www.boost.org/LICENSE_1_0.txt)
 *
 */

#ifndef ANY_H
#define ANY_H

#include <memory>
#include <type_traits>

namespace types
{

class Any
{
public:
    template<typename T>
    using Bare = typename std::decay<T>::type;

    // Templated constructors from other values
    template<typename Forward>
    Any(Forward&& value)
        : content_( new Holder<Bare<Forward>>(std::forward<Forward>(value) ))
    {
    }

    // Other constructors
    Any()
        :content_(nullptr)
    {
    }

    Any(Any& other)
        :Any( const_cast<const Any&>(other))
    {
    }

    Any(const Any& other)
        : content_(other.content_ ? other.content_->clone() : nullptr)
    {
    }

    Any(Any&& other)
        : content_(std::move(other.content_))
    {
    }

    ~Any()
    {
    }

    //Assignment operator
    template<typename Forward>
    Any& operator=(Forward&& value)
    {
        //This will dispatch to correct constructor
        Any newAny( std::forward<Forward>(value) );
        std::swap(newAny.content_, content_ );
        return *this;
    }

    //Helper functions
    bool empty() const
    {
        return !content_;
    }

    const std::type_info & type() const
    {
        return content_ ? content_->type() : typeid(void);
    }

    template<typename Request> bool is() const
    {
        return typeid(Bare<Request>) == type();
    }

    //Getter functions (I don't like friends)
    template<typename Request>
    const Bare<Request>& as() const
    {
        auto ptr = dynamic_cast<Holder<Bare<Request>>*>( content_.get() );
        if (!ptr)
            throw std::bad_cast();

        return ptr->held_;
    }

    template<typename Request>
    Bare<Request>& as()
    {
        const Any& constThis = *this;
        return const_cast<Bare<Request>&>(constThis.as<Request>());
    }

    //Cast operator
    template<typename Request>
    operator Request() const
    {
        return as<Bare<Request>>();
    }

private:
    class Placeholder
    {
    public:
        typedef std::unique_ptr<Placeholder> Ptr;

        virtual ~Placeholder() {}
        virtual const std::type_info & type() const = 0;
        virtual Ptr clone() const = 0;
    };

    template<typename ValueType>
    class Holder : public Placeholder
    {
    public:
        template<typename Forward>
        Holder(Forward&& value)
            : held_( std::forward<Forward>(value) )
        {
        }

        virtual const std::type_info & type() const override
        {
            return typeid(ValueType);
        }

        virtual Placeholder::Ptr clone() const override
        {
            return Placeholder::Ptr(new Holder<ValueType>(held_));
        }

        ValueType held_;
    };

    Placeholder::Ptr content_;
};

}//End of types namespace

#endif // ANY_H
